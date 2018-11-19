#include "trajectory_3Dscans_2_pointcloud_alg_node.h"

Trajectory3DScans2PointcloudAlgNode::Trajectory3DScans2PointcloudAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<Trajectory3DScans2PointcloudAlgorithm>()
{
  //init class attributes if necessary
  emptyPointCloud_ = true;

  double d[4];
  public_node_handle_.param<double>("dx_base_2_h3d", d[0], 0.0);
  public_node_handle_.param<double>("dy_base_2_h3d", d[1], 0.0);
  public_node_handle_.param<double>("dz_base_2_h3d", d[2], 1.0);
  public_node_handle_.param<double>("dth_base_2_h3d", d[3], 0.0);
  T_laser_frame_ = transformation_matrix(d[0], d[1],d[2],d[3]);

  PointCloud_msg_.header.frame_id = "/map";

  ROS_DEBUG("TR 2 3D PC: Config updated");

  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  this->slices3D_pointcloud_publisher_ = this->public_node_handle_.advertise<sensor_msgs::PointCloud2>("slices3D_pointcloud", 1);

  // [init subscribers]
  this->slices3D_subscriber_ = this->public_node_handle_.subscribe("slices3D", 100, &Trajectory3DScans2PointcloudAlgNode::slices3D_callback, this);
  this->trajectory_subscriber_ = this->public_node_handle_.subscribe("trajectory", 100, &Trajectory3DScans2PointcloudAlgNode::trajectory_callback, this);
  
  pthread_mutex_init(&this->last_trajectory_mutex_,NULL);

  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

Trajectory3DScans2PointcloudAlgNode::~Trajectory3DScans2PointcloudAlgNode(void)
{
  // [free dynamic memory]
}

void Trajectory3DScans2PointcloudAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  if (new_trajectory_)
  {
    // Recompute 3D PointCloud
    last_trajectory_mutex_enter();
    iri_poseslam::Trajectory traj = last_trajectory_;
    new_trajectory_ = false;
    last_trajectory_mutex_exit();

    update_pointcloud(traj);
  }
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  this->slices3D_pointcloud_publisher_.publish(this->PointCloud_msg_);
  ROS_DEBUG("TR 2 3D PC: PointCloud_msg_ published! size = %u", PointCloud_msg_.height * PointCloud_msg_.width);
}

/*  [subscriber callbacks] */
void Trajectory3DScans2PointcloudAlgNode::slices3D_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{  
  ROS_DEBUG("TR 2 3D PC New slice Message Received");
  if (trajectory_slices_.size() == trajectory_slices_.capacity())
    trajectory_slices_.reserve(trajectory_slices_.size() + 100);

  trajectory_slices_.push_back(*msg);
}

void Trajectory3DScans2PointcloudAlgNode::trajectory_callback(const iri_poseslam::Trajectory::ConstPtr& msg)
{ 
  ROS_DEBUG("TR 2 3D PC New trajectory Message Received"); 
  last_trajectory_mutex_enter();
  last_trajectory_ = *msg;
  new_trajectory_ = last_trajectory_.poses.size() > 1; // only true from second pose in order to interpolate
  last_trajectory_mutex_exit();
}

void Trajectory3DScans2PointcloudAlgNode::last_trajectory_mutex_enter(void) 
{ 
  pthread_mutex_lock(&this->last_trajectory_mutex_); 
} 

void Trajectory3DScans2PointcloudAlgNode::last_trajectory_mutex_exit(void) 
{ 
  pthread_mutex_unlock(&this->last_trajectory_mutex_); 
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void Trajectory3DScans2PointcloudAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->alg_.unlock();
}

void Trajectory3DScans2PointcloudAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<Trajectory3DScans2PointcloudAlgNode>(argc, argv, "trajectory_3Dscans_2_pointcloud_alg_node");
}

void Trajectory3DScans2PointcloudAlgNode::update_pointcloud(const iri_poseslam::Trajectory& trajectory)
{
  ROS_DEBUG("TR 2 3D PC: Update pointcloud"); 
  // AUX VARIABLES
  bool LoopClosed = (trajectory.loops.size() > last_loops_);
  last_loops_ = trajectory.loops.size();
  uint slice_id, from_id, new_width, new_size, new_fields_size;
  uint N_slices = trajectory_slices_.size();

  // LOOP CLOSED: Recompute all slices
  if (LoopClosed) 
  {
    clear_PointCloud_msg();
    from_id = 0;
    new_width = 0;
    new_size = 0;
    new_fields_size = 0;
    ROS_DEBUG("TR 2 3D PC: Recompute all %u slices of the trajectory with %u steps", N_slices, uint(trajectory.poses.size()));
  }

  // NOT LOOP CLOSED: Add only the last slices
  else
  {
    from_id = interpolation_buffer_.size(); // last slices are the ones that doesn't have the interpolation computed
    new_width = PointCloud_msg_.width;
    new_size = PointCloud_msg_.data.size();
    new_fields_size = PointCloud_msg_.fields.size();
  }

  // transform slices pointcloud
  slice_id = from_id;
  while (slice_id < N_slices && trajectory_slices_.at(slice_id).header.stamp < trajectory.poses.back().header.stamp)
  {
    if  (slice_id >= interpolation_buffer_.size())
    {
      // Compute and add the interpolation index and factor if there is not
      uint last_id = (interpolation_buffer_.size() > 0 ? interpolation_buffer_.back().first : 0);
      if (interpolation_buffer_.size() == interpolation_buffer_.capacity())
        interpolation_buffer_.reserve(interpolation_buffer_.size() + 100);
      interpolation_buffer_.push_back(search_interpolation(trajectory_slices_.at(slice_id), last_id));
    }
    // transform slices pointcloud
    transform_point_cloud(trajectory_slices_.at(slice_id), interpole_slice_pose(slice_id));
    new_width += trajectory_slices_.at(slice_id).width;
    new_size += trajectory_slices_.at(slice_id).data.size();
    new_fields_size += trajectory_slices_.at(slice_id).fields.size();
    slice_id++;
  }

  // add transformed slices to pointcloud
  uint fields_id = PointCloud_msg_.fields.size();
  uint data_id = PointCloud_msg_.data.size();
  PointCloud_msg_.width = new_width;
  PointCloud_msg_.fields.resize(new_fields_size);
  PointCloud_msg_.data.resize(new_size);
  for (uint i = from_id; i<slice_id; i++)
  {
    std::copy(trajectory_slices_.at(i).fields.begin(), trajectory_slices_.at(i).fields.end(), PointCloud_msg_.fields.begin() + fields_id);
    std::copy(trajectory_slices_.at(i).data.begin(), trajectory_slices_.at(i).data.end(), PointCloud_msg_.data.begin() + data_id);
    fields_id += trajectory_slices_.at(i).fields.size();
    data_id += trajectory_slices_.at(i).data.size();
  }
  PointCloud_msg_.header = trajectory_slices_.back().header;
  PointCloud_msg_.header.frame_id = "/map";
  PointCloud_msg_.height = trajectory_slices_.back().height;
  PointCloud_msg_.is_bigendian = trajectory_slices_.back().is_bigendian;
  PointCloud_msg_.point_step = trajectory_slices_.back().point_step;
  PointCloud_msg_.row_step = trajectory_slices_.back().row_step;
  PointCloud_msg_.is_dense = trajectory_slices_.back().is_dense;
}

void Trajectory3DScans2PointcloudAlgNode::transform_point_cloud(sensor_msgs::PointCloud2& pcloud, const geometry_msgs::Pose& pose) 
{
  Matrix4f T_pose = transformation_matrix(pose.position.x, pose.position.y, pose.position.z, tf::getYaw(pose.orientation));

  pcl_ros::transformPointCloud(T_pose * T_laser_frame_, pcloud, pcloud);
}

void Trajectory3DScans2PointcloudAlgNode::add_to_PointCloud_msg(const sensor_msgs::PointCloud2& newPointCloud)
{
  if (emptyPointCloud_)
  {
    PointCloud_msg_ = newPointCloud;
    PointCloud_msg_.header.frame_id = "/map";
    emptyPointCloud_ = false;
  }
  else
  {
    PointCloud_msg_.width = PointCloud_msg_.width + newPointCloud.width;
    PointCloud_msg_.fields.resize(PointCloud_msg_.fields.size() + newPointCloud.fields.size());
    std::copy_backward(newPointCloud.fields.begin(), newPointCloud.fields.end(), PointCloud_msg_.fields.end());
    
    PointCloud_msg_.data.resize(PointCloud_msg_.data.size() + newPointCloud.data.size());
    std::copy_backward(newPointCloud.data.begin(), newPointCloud.data.end(), PointCloud_msg_.data.end());

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ> newcloud;

    // pcl::fromROSMsg(PointCloud_msg_, *cloud1);
    // pcl::fromROSMsg(newPointCloud, *cloud2);
  
    // newcloud = *cloud1 + *cloud2;

    // pcl::toROSMsg(newcloud, PointCloud_msg_);
    // PointCloud_msg_.header.frame_id = "/map";
  }
}

void Trajectory3DScans2PointcloudAlgNode::clear_PointCloud_msg()
{
  PointCloud_msg_.height = 0;
  PointCloud_msg_.width = 0;
  PointCloud_msg_.data.clear();
  PointCloud_msg_.fields.clear();
  emptyPointCloud_ = true;
}

geometry_msgs::Pose Trajectory3DScans2PointcloudAlgNode::interpole_slice_pose(const uint& id) const
{
  //ROS_INFO("TR 2 3D PC: Interpolation of slice %u. interpolation_buffer_.size() = %lu", id, interpolation_buffer_.size());

  // GET THE INTERPOLATION INDEX AND FACTOR
  if (id >= interpolation_buffer_.size())
    ROS_ERROR("TR 2 3D PC: Index higher than the interpolation buffer size!");

  uint id_pre  = interpolation_buffer_.at(id).first;
  double alpha = interpolation_buffer_.at(id).second;

  // GET THE PRE AND POST POSES
  geometry_msgs::Point pre_position  = last_trajectory_.poses.at(id_pre).pose.pose.position;
  geometry_msgs::Point post_position = last_trajectory_.poses.at(id_pre + 1).pose.pose.position;
  double pre_yaw = tf::getYaw(last_trajectory_.poses.at(id_pre).pose.pose.orientation);
  double post_yaw = tf::getYaw(last_trajectory_.poses.at(id_pre + 1).pose.pose.orientation);
  double d_yaw = pi_2_pi(post_yaw- pre_yaw);

  // INTERPOLE
  geometry_msgs::Pose slice_pose;
  slice_pose.position.x = (1 - alpha) * pre_position.x + alpha * post_position.x;
  slice_pose.position.y = (1 - alpha) * pre_position.y + alpha * post_position.y;
  slice_pose.position.z = (1 - alpha) * pre_position.z + alpha * post_position.z;
  slice_pose.orientation = tf::createQuaternionMsgFromYaw(pre_yaw + alpha * d_yaw);
  
  return slice_pose;
}

std::pair<uint, double> Trajectory3DScans2PointcloudAlgNode::search_interpolation(const sensor_msgs::PointCloud2& slice, const uint& prev_i) const
{
  // SEARCH THE INTERPOLATION INDEX
  bool found = false;
  uint i = std::max(prev_i,uint(1)); //start from pose 1 in order to be able to interpolate with a previous pose
  uint id = 0;
  // search
  while (!found && i < last_trajectory_.poses.size())
  {
    if (slice.header.stamp < last_trajectory_.poses.at(i).header.stamp)
    {
      id  = i - 1;
      found = true;
    }
    else
      i++;
  }

  if (!found)
    ROS_ERROR("TR 2 3D PC: Interpolation index not found");

  // COMPUTE THE INTERPOLATION FACTOR
  ros::Time pre_time  = last_trajectory_.poses.at(id).header.stamp;
  ros::Time post_time = last_trajectory_.poses.at(id + 1).header.stamp;
  double alpha = (slice.header.stamp - pre_time).toSec() / (post_time - pre_time).toSec();

  std::pair<uint, double> result(id, alpha);
  return result;
}

Matrix4f Trajectory3DScans2PointcloudAlgNode::transformation_matrix(const float x, const float y, const float z, const float alpha) const
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

double Trajectory3DScans2PointcloudAlgNode::pi_2_pi(const double& angle) const
{
  return angle - 2 * M_PI * floor((angle + M_PI)/(2 * M_PI));
}