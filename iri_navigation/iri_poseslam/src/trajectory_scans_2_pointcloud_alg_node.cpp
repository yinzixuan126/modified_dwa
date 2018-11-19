#include "trajectory_scans_2_pointcloud_alg_node.h"

TrajectoryScans2PointcloudAlgNode::TrajectoryScans2PointcloudAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<TrajectoryScans2PointcloudAlgorithm>()
{
  //init class attributes if necessary
  emptyPointCloud_ = true;
  new_trajectory_ = false;

  public_node_handle_.param<bool>("publish_redundant", publish_redundant_, true);
  public_node_handle_.param<std::string>("base_frame_id", base_frame_id_, "/teo/base_link");
  public_node_handle_.param<std::string>("laser_frame_id", laser_frame_id_, "/teo/front_laser");

  ROS_DEBUG("TR 2 PC: Config updated");
 

  PointCloud_msg_.header.frame_id = "/map";
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  this->laser_pointcloud_publisher_ = this->public_node_handle_.advertise<sensor_msgs::PointCloud2>("laser_pointcloud", 1);
  
  // [init subscribers]
  this->scan_subscriber_ = this->public_node_handle_.subscribe("scan", 10, &TrajectoryScans2PointcloudAlgNode::scan_callback, this);
  this->trajectory_subscriber_ = this->public_node_handle_.subscribe("trajectory", 1, &TrajectoryScans2PointcloudAlgNode::trajectory_callback, this);
  
  pthread_mutex_init(&this->last_trajectory_mutex_,NULL);

  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
  tf_ready_ = false;
  T_base_laser_.setIdentity();
  T_laser_base_.setIdentity();
  load_tf();
}

TrajectoryScans2PointcloudAlgNode::~TrajectoryScans2PointcloudAlgNode(void)
{
  // [free dynamic memory]
}

void TrajectoryScans2PointcloudAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  if (!tf_ready_)
    load_tf();
  
  if (new_trajectory_)
  {
    last_trajectory_mutex_enter();
    iri_poseslam::Trajectory trajectory = last_trajectory_;
    new_trajectory_ = false;
    last_trajectory_mutex_exit();

    // Update the trajecory scans buffer
    update_trajectory_scans(trajectory);
  
    // Recompute occupancy grid
    update_pointcloud(trajectory);
  }

  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  this->laser_pointcloud_publisher_.publish(this->PointCloud_msg_);
  ROS_DEBUG("TR 2 PC: PointCloud_msg_ published! size = %u", PointCloud_msg_.height * PointCloud_msg_.width);
}

/*  [subscriber callbacks] */
void TrajectoryScans2PointcloudAlgNode::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{  
  //ROS_INFO("TR 2 PC New LaserScan Message Received"); 
  laser_scan_buffer_.push(*msg);
}

void TrajectoryScans2PointcloudAlgNode::trajectory_callback(const iri_poseslam::Trajectory::ConstPtr& msg)
{ 
  last_trajectory_mutex_enter();
  if (publish_redundant_ || msg->steps_2_states.back() != -1)
  {
    last_trajectory_ = *msg;
    new_trajectory_ = true;
  }
  last_trajectory_mutex_exit();
}

void TrajectoryScans2PointcloudAlgNode::last_trajectory_mutex_enter(void) 
{ 
  pthread_mutex_lock(&this->last_trajectory_mutex_); 
} 

void TrajectoryScans2PointcloudAlgNode::last_trajectory_mutex_exit(void) 
{ 
  pthread_mutex_unlock(&this->last_trajectory_mutex_); 
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void TrajectoryScans2PointcloudAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock(); 
  this->alg_.unlock();
}

void TrajectoryScans2PointcloudAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<TrajectoryScans2PointcloudAlgNode>(argc, argv, "trajectory_scans_2_pointcloud_alg_node");
}

void TrajectoryScans2PointcloudAlgNode::update_trajectory_scans(const iri_poseslam::Trajectory& trajectory)
{
  // Reserve space in buffer
  if (trajectory_scans_.size() == trajectory_scans_.capacity())
    trajectory_scans_.reserve(trajectory_scans_.size() + 100);

  // Add the lasers of the new trajectory poses
  uint N_poses = (publish_redundant_ ? trajectory.poses.size() : trajectory.states_2_steps.size());

  ROS_DEBUG("TR 2 PC: update traj scans: N_poses = %i - trajectory_scans_.size() = %lu", N_poses, trajectory_scans_.size());

  while (N_poses > trajectory_scans_.size())
  {
    uint step_idx = ( publish_redundant_ ? trajectory_scans_.size() :  trajectory.states_2_steps.at(trajectory_scans_.size()) );
    if (laser_scan_buffer_.size() > 0 && laser_scan_buffer_.front().header.stamp == trajectory.poses.at(step_idx).header.stamp)
    {
      trajectory_scans_.push_back(laser_scan_buffer_.front());
      laser_scan_buffer_.pop();
      //ROS_INFO("TR 2 PC: laser scan added! \n\tnSteps = %i \n\tnStates = %i \n\ttrajectory_scans_.size = %i", trajectory.steps_2_states.size(), trajectory.states_2_steps.size(), trajectory_scans_.size());
    }
    else if (laser_scan_buffer_.size() > 0 && laser_scan_buffer_.front().header.stamp < trajectory.poses.at(step_idx).header.stamp)
      laser_scan_buffer_.pop();
    else
    {
      ROS_WARN("TR 2 PC: Any trajectory pose laser scan lost!");
      sensor_msgs::LaserScan empty_LaserScan;
      empty_LaserScan.angle_increment = 0;
      empty_LaserScan.header = trajectory.poses.at(step_idx).header;
      trajectory_scans_.push_back(empty_LaserScan);
    }
  }
}

void TrajectoryScans2PointcloudAlgNode::update_pointcloud(const iri_poseslam::Trajectory& trajectory)
{
  bool LoopClosed = (trajectory.loops.size() > last_loops_);
  last_loops_ = trajectory.loops.size();
  uint from_step;

  // LOOP CLOSED: Recompute pointcloud from zero
  if (LoopClosed)
  {
    clear_PointCloud_msg();
    from_step = 0;
    ROS_DEBUG("TR 2 PC: Loop Closed! Recompute all %u poses of the trajectory with %u laser scans", uint(trajectory.poses.size()), uint(trajectory_scans_.size()));
  }
  
  // NOT LOOP CLOSED: Add the last scans to pointcloud
  else
    from_step = last_step_ + 1;

  // Update the pointcloud
  for (uint i = from_step; i < trajectory.poses.size(); i++)
  {
    //ROS_INFO("TR 2 PC: scan step: %i\ntrajectory.poses.size() = %lu\ntrajectory.steps_2_states.size() = %lu", i, trajectory.poses.size(), trajectory.steps_2_states.size());
    if (publish_redundant_ || trajectory.steps_2_states.at(i) != -1)
    {
      ROS_DEBUG("TR 2 PC: Adding scan from pose: %u - %f, %f, %f", i,
                           trajectory.poses.at(i).pose.pose.position.x, 
                           trajectory.poses.at(i).pose.pose.position.y, 
                           tf::getYaw(trajectory.poses.at(i).pose.pose.orientation));
      sensor_msgs::LaserScan laser_scan = ( publish_redundant_ ? trajectory_scans_.at(i) : trajectory_scans_.at(trajectory.steps_2_states.at(i)) );
      if (laser_scan.header.stamp == trajectory.poses.at(i).header.stamp && laser_scan.angle_increment != 0)
        add_to_PointCloud_msg(laser_scan_to_point_cloud(laser_scan, trajectory.poses.at(i).pose.pose));

      else
        ROS_ERROR("TR 2 PC: headers don't match! step %i", i);
    }
  }
  last_step_ = trajectory.poses.size() - 1;
}

sensor_msgs::PointCloud2 TrajectoryScans2PointcloudAlgNode::laser_scan_to_point_cloud(const sensor_msgs::LaserScan& LScan, const geometry_msgs::Pose& pose) 
{
  sensor_msgs::PointCloud2 pcloud;
  sensor_msgs::PointCloud2 transformed_pcloud;
  
  laser_projector_.projectLaser(LScan, pcloud, LScan.range_max - 1e-6);
  
  Matrix4f T_map_base = transformation_matrix(pose.position.x, pose.position.y, pose.position.z, tf::getYaw(pose.orientation));
  Eigen::Matrix4f T_base_laser;
  pcl_ros::transformAsMatrix(T_base_laser_, T_base_laser);

  pcl_ros::transformPointCloud(T_map_base * T_base_laser, pcloud, transformed_pcloud);

  return transformed_pcloud;
}

void TrajectoryScans2PointcloudAlgNode::add_to_PointCloud_msg(const sensor_msgs::PointCloud2& newPointCloud)
{
  if (emptyPointCloud_)
  {
    PointCloud_msg_ = newPointCloud;
    emptyPointCloud_ = false;
    PointCloud_msg_.header.frame_id = "/map";
  }
  else
  {
    PointCloud_msg_.width = PointCloud_msg_.width + newPointCloud.width;
    PointCloud_msg_.fields.resize(PointCloud_msg_.fields.size() + newPointCloud.fields.size());
    std::copy_backward(newPointCloud.fields.begin(), newPointCloud.fields.end(), PointCloud_msg_.fields.end());
    
    PointCloud_msg_.data.resize(PointCloud_msg_.data.size() + newPointCloud.data.size());
    std::copy_backward(newPointCloud.data.begin(), newPointCloud.data.end(), PointCloud_msg_.data.end());
  }
  // if (emptyPointCloud_)
  // {
  //   PointCloud_msg_ = newPointCloud;
  //   PointCloud_msg_.header.frame_id = "/map";
  //   emptyPointCloud_ = false;
  // }
  // else
  // {
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  //   pcl::PointCloud<pcl::PointXYZ> newcloud;

  //   pcl::fromROSMsg(PointCloud_msg_, *cloud1);
  //   pcl::fromROSMsg(newPointCloud, *cloud2);
  
  //   newcloud = *cloud1 + *cloud2;

  //   pcl::toROSMsg(newcloud, PointCloud_msg_);
  //   PointCloud_msg_.header.frame_id = "/map";
  // }
}

void TrajectoryScans2PointcloudAlgNode::clear_PointCloud_msg()
{
  PointCloud_msg_.height = 0;
  PointCloud_msg_.width = 0;
  PointCloud_msg_.data.clear();
  PointCloud_msg_.fields.clear();
  emptyPointCloud_ = true;
}

Matrix4f TrajectoryScans2PointcloudAlgNode::transformation_matrix(const float x, const float y, const float z, const float alpha) const
{
  Matrix4f T = MatrixXf::Identity(4,4);

  // Rotation
  T(0,0) =  cos(alpha);
  T(0,1) = -sin(alpha);
  T(1,0) =  sin(alpha);
  T(1,1) =  cos(alpha);

  // Translation
  T(0,3) = x;
  T(1,3) = y;
  T(2,3) = z;

  return T;
}


void TrajectoryScans2PointcloudAlgNode::load_tf()
{
  tf::TransformListener tfl;
  tf::StampedTransform T_base_laser_stamped;
  try
  {
    tfl.waitForTransform(base_frame_id_, laser_frame_id_, ros::Time::now(), ros::Duration(1.0));
    tfl.lookupTransform(base_frame_id_, laser_frame_id_, ros::Time::now(), T_base_laser_stamped);
    T_base_laser_ = T_base_laser_stamped;
    T_laser_base_ = T_base_laser_.inverse();
    tf_ready_ = true;
  }
  catch (tf::TransformException ex)
  {
    //ROS_ERROR("SENSORS 2 : Transform exception: %s",ex.what());
  }
}
