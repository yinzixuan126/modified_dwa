#include "diff_platform_simulator_alg_node.h"

DiffPlatformSimulatorAlgNode::DiffPlatformSimulatorAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<DiffPlatformSimulatorAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 50;//in [Hz]

  //ROS_INFO("DiffPlatformSimulatorAlgNode:: this->tf_prefix_.c_str()=%s",this->tf_prefix_.c_str()); 

  this->public_node_handle_.getParam("robot", this->tf_prefix_);
  this->odom_id_           = "/" + this->tf_prefix_ + "/odom";
  this->base_link_id_      = "/" + this->tf_prefix_ + "/base_link";
  this->base_footprint_id_ = "/" + this->tf_prefix_ + "/base_footprint";
  this->linearX = 0.0;
  this->angularZ= 0.0;

  // [init publishers]
  this->odom_publisher_ = this->public_node_handle_.advertise<nav_msgs::Odometry>("odom", 10);
  
  // [init subscribers]
  this->cmd_vel_subscriber_ = this->public_node_handle_.subscribe("cmd_vel", 10, &DiffPlatformSimulatorAlgNode::cmd_vel_callback, this);
  pthread_mutex_init(&this->cmd_vel_mutex_,NULL);

  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

DiffPlatformSimulatorAlgNode::~DiffPlatformSimulatorAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->cmd_vel_mutex_);
}

void DiffPlatformSimulatorAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  // Initialize the topic message structure
  cmd_vel_mutex_enter();
  static double last_vt=0.0;
  static double last_vr=0.0;

  double vt=last_vt; // this->linearX;
  double vt_step=0.1;
  double vr=last_vr; // this->angularZ;
  double vr_step=0.05;
  if(vt<this->linearX)
  {
    vt+=vt_step;
    if(vt>this->linearX)
      vt=this->linearX;
  }
  else
  {
    vt-=vt_step;
    if(vt<this->linearX)
      vt=this->linearX;
  }
  last_vt=vt;

  if(vr<this->angularZ)
  {
    vr+=vr_step;
    if(vr>this->angularZ)
      vr=this->angularZ;
  }
  else
  {
    vr-=vr_step;
    if(vr<this->angularZ)
      vr=this->angularZ;
  }
  last_vr=vr;
  this->cmd_vel_mutex_exit();

  ros::Time current_time     = ros::Time::now();
  static ros::Time last_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();

  static double accum_th = 0;

  double vx = vt*cos(accum_th);
  double vy = vt*sin(accum_th);
  accum_th += vr*dt;//update de la pose posterior a la propagacion

  double delta_x  = vx*dt;
  double delta_y  = vy*dt;

  double pitch_angle=0.0;
  if(vt>0)
    pitch_angle=0.1*vt;
  else if(vt<0)
    pitch_angle=-0.1*vt;
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(accum_th);
  geometry_msgs::Quaternion odom_quat_pitch = tf::createQuaternionMsgFromRollPitchYaw(0.0,pitch_angle,0.0);

  this->transform.translation.x += delta_x;
  this->transform.translation.y += delta_y;
  this->transform.translation.z += 0.0;
  this->transform.rotation       = odom_quat;

  //update transform pitch message
  transform_pitch.translation.x = 0.0;
  this->transform_pitch.translation.y = 0.0;
  this->transform_pitch.translation.z = 0.23; //wheel radius
  this->transform_pitch.rotation      = odom_quat_pitch;

  pose.pose.position.x += delta_x;
  pose.pose.position.y += delta_y;
  pose.pose.position.z += 0.0;
  pose.pose.orientation = odom_quat;

  twist.twist.linear.x  = vt;
  twist.twist.linear.y  = 0.0;
  twist.twist.linear.z  = 0.0;
  twist.twist.angular.x = 0.0;
  twist.twist.angular.y = 0.0;
  twist.twist.angular.z = vr;

  last_time = current_time;

  //send the transform
  geometry_msgs::TransformStamped odom_trans_msg;
  //first, publish transform over tf
  odom_trans_msg.header.stamp    = current_time;
  odom_trans_msg.header.frame_id = this->odom_id_;
  odom_trans_msg.child_frame_id  = this->base_footprint_id_;
  odom_trans_msg.transform       = this->transform;
  this->odom_broadcaster.sendTransform(odom_trans_msg);

  odom_trans_msg.header.stamp    = current_time;
  odom_trans_msg.header.frame_id = this->base_footprint_id_;
  odom_trans_msg.child_frame_id  = this->base_link_id_;
  odom_trans_msg.transform       = this->transform_pitch;
  this->odom_broadcaster.sendTransform(odom_trans_msg);

  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  // Uncomment the following line to publish the topic message
  this->Odometry_msg_.header.stamp     = current_time;
  this->Odometry_msg_.header.frame_id  = this->odom_id_;
  this->Odometry_msg_.child_frame_id   = this->base_link_id_;
  this->Odometry_msg_.pose             = pose;
  this->Odometry_msg_.twist            = twist;
  this->odom_publisher_.publish(this->Odometry_msg_);
  

}

/*  [subscriber callbacks] */
void DiffPlatformSimulatorAlgNode::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  //ROS_INFO("DiffPlatformSimulatorAlgNode::cmd_vel_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  this->cmd_vel_mutex_enter();
  this->linearX  = msg->linear.x;
  this->angularZ = msg->angular.z;
  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  this->cmd_vel_mutex_exit();
}

void DiffPlatformSimulatorAlgNode::cmd_vel_mutex_enter(void)
{
  pthread_mutex_lock(&this->cmd_vel_mutex_);
}

void DiffPlatformSimulatorAlgNode::cmd_vel_mutex_exit(void)
{
  pthread_mutex_unlock(&this->cmd_vel_mutex_);
}


/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void DiffPlatformSimulatorAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void DiffPlatformSimulatorAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<DiffPlatformSimulatorAlgNode>(argc, argv, "diff_platform_simulator_alg_node");
}
