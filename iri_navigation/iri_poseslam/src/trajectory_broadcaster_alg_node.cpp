#include "trajectory_broadcaster_alg_node.h"

TrajectoryBroadcasterAlgNode::TrajectoryBroadcasterAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<TrajectoryBroadcasterAlgorithm>()
{
  //init class attributes if necessary
  loop_rate_ = 10;//in [Hz]
  
  recompute_ = false;  
  public_node_handle_.param<std::string>("base_frame_id", base_frame_id_, "/teo/base_link");
  public_node_handle_.param<std::string>("odom_frame_id", odom_frame_id_, "/teo/odom");
  public_node_handle_.param<std::string>("map_frame_id",  map_frame_id_, "/map");
  T_map_odom_ = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0, 0.0));
  tfb_.sendTransform( tf::StampedTransform(T_map_odom_, ros::Time::now(), map_frame_id_, odom_frame_id_));

  // [init publishers]

  // [init subscribers]
  trajectory_subscriber_ = public_node_handle_.subscribe("trajectory", 1, &TrajectoryBroadcasterAlgNode::trajectory_callback, this);
  
  pthread_mutex_init(&this->recompute_tf_mutex_,NULL);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

TrajectoryBroadcasterAlgNode::~TrajectoryBroadcasterAlgNode(void)
{
  // [free dynamic memory]
}

void TrajectoryBroadcasterAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  if (recompute_)
  {
    recompute_tf_mutex_enter();
    geometry_msgs::PoseWithCovarianceStamped pose = new_pose_;
    recompute_ = false;
    recompute_tf_mutex_exit();

    try{
      // T_base_odom_ via TF
      tf::StampedTransform T_base_odom_stamped;
      tfl_.waitForTransform(base_frame_id_, odom_frame_id_, pose.header.stamp, ros::Duration(1.0));
      tfl_.lookupTransform(base_frame_id_, odom_frame_id_, pose.header.stamp, T_base_odom_stamped);
      T_base_odom_ = T_base_odom_stamped;

      // T_map_base_
      //ROS_INFO("TRAJ BROADCASTER: Pose:\n\tPosition\tx: %f - y: %f - z: %f\n\tOrientation\tx: %f - y: %f - z: %f - w: %f",msg->poses.back().pose.pose.position.x,msg->poses.back().pose.pose.position.y,msg->poses.back().pose.pose.position.z,msg->poses.back().pose.pose.orientation.x,msg->poses.back().pose.pose.orientation.y,msg->poses.back().pose.pose.orientation.z,msg->poses.back().pose.pose.orientation.w);
      tf::poseMsgToTF(pose.pose.pose, T_map_base_);
      
      // T_map_odom_
      T_map_odom_ = T_map_base_ * T_base_odom_;
    }
    catch (tf::TransformException ex){
      //ROS_ERROR("TRAJ BROADCASTER: Transform exception: %s",ex.what());
    }
  }
  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  tfb_.sendTransform( tf::StampedTransform(T_map_odom_, ros::Time::now(), map_frame_id_, odom_frame_id_) );
  ROS_DEBUG("TRAJ BROADCASTER: transform sended"); 
}

/*  [subscriber callbacks] */
void TrajectoryBroadcasterAlgNode::trajectory_callback(const iri_poseslam::Trajectory::ConstPtr& msg)
{ 
  //ROS_INFO("TRAJ BROADCASTER: New Trajectory Message Received");
  recompute_tf_mutex_enter();
  new_pose_ = msg->poses.back();
  recompute_ = true;
  recompute_tf_mutex_exit();
}

void TrajectoryBroadcasterAlgNode::recompute_tf_mutex_enter(void) 
{ 
  pthread_mutex_lock(&this->recompute_tf_mutex_); 
} 

void TrajectoryBroadcasterAlgNode::recompute_tf_mutex_exit(void) 
{ 
  pthread_mutex_unlock(&this->recompute_tf_mutex_); 
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void TrajectoryBroadcasterAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->alg_.unlock();
}

void TrajectoryBroadcasterAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<TrajectoryBroadcasterAlgNode>(argc, argv, "trajectory_broadcaster_alg_node");
}