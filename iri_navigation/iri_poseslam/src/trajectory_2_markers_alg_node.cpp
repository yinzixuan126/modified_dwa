#include "trajectory_2_markers_alg_node.h"

Trajectory2MarkersAlgNode::Trajectory2MarkersAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<Trajectory2MarkersAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]
  new_trajectory_ = false;

  // trajectory line initialization
  trajectory_marker_.header.stamp = ros::Time::now();
  trajectory_marker_.header.frame_id = "/map";
  trajectory_marker_.type = visualization_msgs::Marker::LINE_STRIP;
  trajectory_marker_.action = visualization_msgs::Marker::ADD;
  trajectory_marker_.ns = "/trajectory";
  trajectory_marker_.id = 1;

  double c[4]; //auxiliar color variable
  public_node_handle_.param<double>("line_color_r", c[0], 0.0);
  public_node_handle_.param<double>("line_color_g", c[1], 0.5);
  public_node_handle_.param<double>("line_color_b", c[2], 0.8);
  public_node_handle_.param<double>("line_color_a", c[3], 0.8);
  public_node_handle_.param<double>("line_width", trajectory_marker_.scale.x, 0.05);
  trajectory_marker_.color.r = c[0];
  trajectory_marker_.color.g = c[1];
  trajectory_marker_.color.b = c[2];
  trajectory_marker_.color.a = c[3];
  
  // loop line initialization
  loops_marker_.header.stamp = ros::Time::now();
  loops_marker_.header.frame_id = "/map";
  loops_marker_.type = visualization_msgs::Marker::LINE_LIST;
  loops_marker_.action = visualization_msgs::Marker::ADD;
  loops_marker_.ns = "/loops";
  loops_marker_.id = 2;
  
  public_node_handle_.param<double>("line_loop_color_r", c[0], 1);
  public_node_handle_.param<double>("line_loop_color_g", c[1], 0.8);
  public_node_handle_.param<double>("line_loop_color_b", c[2], 0.0);
  public_node_handle_.param<double>("line_loop_color_a", c[3], 0.5);
  public_node_handle_.param<double>("line_loop_width", loops_marker_.scale.x, 0.07);
  loop_color_.r = c[0];
  loop_color_.g = c[1];
  loop_color_.b = c[2];
  loop_color_.a = c[3];
  loops_marker_.color = loop_color_;
  
  // current marker initialization
  current_marker_.header.stamp = ros::Time::now();
  current_marker_.header.frame_id = "/map";
  current_marker_.type = visualization_msgs::Marker::SPHERE;
  current_marker_.action = visualization_msgs::Marker::ADD;
  current_marker_.ns = "/current";
  current_marker_.id = 3;
  
  public_node_handle_.param<double>("current_marker_color_r", c[0], 0.8);
  public_node_handle_.param<double>("current_marker_color_g", c[1], 0.0);
  public_node_handle_.param<double>("current_marker_color_b", c[2], 1);
  public_node_handle_.param<double>("current_marker_color_a", c[3], 0.8);
  current_marker_.color.r = c[0];
  current_marker_.color.g = c[1];
  current_marker_.color.b = c[2];
  current_marker_.color.a = c[3];
  
  // Covariance markers color
  public_node_handle_.param<double>("covariance_color_r", c[0], 0.0);
  public_node_handle_.param<double>("covariance_color_g", c[1], 0.5);
  public_node_handle_.param<double>("covariance_color_b", c[2], 0.8);
  public_node_handle_.param<double>("covariance_color_a", c[3], 0.5);
  covariance_color_.r = c[0];
  covariance_color_.g = c[1];
  covariance_color_.b = c[2];
  covariance_color_.a = c[3];

  // Variables initialization
  nLoops_ = 0;
  
  ROS_DEBUG("TR 2 MARKERS: Config updated");

  // [init publishers]
  this->CovarianceMarkers_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("CovarianceMarkers", 1);
  this->TrajectoryMarkers_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("TrajectoryMarkers", 1);
  this->CurrentPoseMarker_publisher_ = this->public_node_handle_.advertise<visualization_msgs::Marker>("CurrentPoseMarker", 1);
  
  // [init subscribers]
  this->trajectory_subscriber_ = this->public_node_handle_.subscribe("trajectory", 1, &Trajectory2MarkersAlgNode::trajectory_callback, this);
  
  pthread_mutex_init(&this->last_trajectory_mutex_,NULL);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

Trajectory2MarkersAlgNode::~Trajectory2MarkersAlgNode(void)
{
  // [free dynamic memory]
}

void Trajectory2MarkersAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  if (new_trajectory_)
  {
    last_trajectory_mutex_enter();
    iri_poseslam::Trajectory traj = last_trajectory_;
    new_trajectory_ = false;
    last_trajectory_mutex_exit();

    update_markers(traj);
  }
  // [fill srv structure and make request to the server]
  // [fill action structure and make request to the action server]
  // [publish messages]
  this->CovarianceMarkers_publisher_.publish(covariance_markers_);
  this->TrajectoryMarkers_publisher_.publish(get_trajectory_marker());
  this->CurrentPoseMarker_publisher_.publish(current_marker_);
}

/*  [subscriber callbacks] */
void Trajectory2MarkersAlgNode::trajectory_callback(const iri_poseslam::Trajectory::ConstPtr& msg)
{ 
  //ROS_INFO("Trajectory2MarkersAlgNode::trajectory_callback: New Message Received");
  last_trajectory_mutex_enter();
  last_trajectory_ = (*msg);
  new_trajectory_ = true;
  last_trajectory_mutex_exit();
}

void Trajectory2MarkersAlgNode::last_trajectory_mutex_enter(void) 
{ 
  pthread_mutex_lock(&this->last_trajectory_mutex_); 
} 

void Trajectory2MarkersAlgNode::last_trajectory_mutex_exit(void) 
{ 
  pthread_mutex_unlock(&this->last_trajectory_mutex_); 
}

/*  [service callbacks] */
/*  [action callbacks] */
/*  [action requests] */

void Trajectory2MarkersAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->alg_.unlock();
}

void Trajectory2MarkersAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<Trajectory2MarkersAlgNode>(argc, argv, "trajectory_2_markers_alg_node");
}

// Trajectory2MarkersAlgNode Public API
void Trajectory2MarkersAlgNode::update_markers(const iri_poseslam::Trajectory& trajectory)
{
  ROS_DEBUG("TR 2 MARKERS: update markers");
  
  // Update current marker
  change_current_marker(get_cov(trajectory.poses.back()), get_theta_cov(trajectory.poses.back()), trajectory.poses.back().pose.pose.position);

  // Update the rest of markers
  uint from_step;

  // LoopClosure
  if (trajectory.loops.size() > nLoops_ * 2)
  {
    // Step from which update covariance and trajectory markers
    from_step = 0;
    // Update nLoops
    nLoops_ = uint(trajectory.loops.size()) / 2;

    // Clear all markers
    covariance_markers_.markers.clear();
    trajectory_marker_.points.clear();
    loops_marker_.points.clear();
    covariance_markers_.markers.reserve(trajectory.states_2_steps.size() + 100);
    trajectory_marker_.points.reserve(trajectory.poses.size() + 100);
    loops_marker_.points.reserve(nLoops_);

    // Recompute loop markers and boolean loop vector
    loop_step_.clear();
    loop_step_.resize(trajectory.poses.size(),false);
    for (uint i = 0; i < nLoops_; i++)
    {
      uint from = trajectory.loops.at(2 * i);
      uint with = trajectory.loops.at(2 * i + 1);
      loops_marker_.points.push_back(trajectory.poses.at(from).pose.pose.position);
      loops_marker_.points.push_back(trajectory.poses.at(with).pose.pose.position);
      loop_step_.at(from) = true;
      loop_step_.at(with) = true;
    }
  }
  else
  {
    // Step from which update covariance and trajectory markers
    from_step = trajectory_marker_.points.size();

    // Reserve space
    if (loop_step_.capacity() == loop_step_.size())
      loop_step_.reserve(loop_step_.size() + 100);

    if (covariance_markers_.markers.capacity() == covariance_markers_.markers.size()) 
      covariance_markers_.markers.reserve(covariance_markers_.markers.size() + 100);

    if (trajectory_marker_.points.capacity() == trajectory_marker_.points.size()) 
      trajectory_marker_.points.reserve(trajectory_marker_.points.size() + 100);
    
    while (loop_step_.size() < trajectory.poses.size())
      loop_step_.push_back(false);
  }

  // Update covariance and trajectory markers
  for (uint i = from_step; i < trajectory.poses.size(); i++)
  {
    // COVARIANCE (only states)
    if (trajectory.steps_2_states.at(i) != -1)
      covariance_markers_.markers.push_back(create_marker(i, trajectory.poses.at(i).header, get_ith_cov(trajectory, i), get_ith_theta_cov(trajectory, i), trajectory.poses.at(i).pose.pose.position, loop_step_.at(i)));

    // TRAJECTORY (all steps)
    trajectory_marker_.points.push_back(trajectory.poses.at(i).pose.pose.position);
  }
  // Headers update
  loops_marker_.header = trajectory.header;
  trajectory_marker_.header = trajectory.header;
}

visualization_msgs::MarkerArray Trajectory2MarkersAlgNode::get_trajectory_marker() const
{
  visualization_msgs::MarkerArray trajectory_markers_array_;
  trajectory_markers_array_.markers.push_back(loops_marker_);
  trajectory_markers_array_.markers.push_back(trajectory_marker_);
  
  return trajectory_markers_array_;
}

visualization_msgs::Marker Trajectory2MarkersAlgNode::create_marker(const uint& id, const std_msgs::Header& header, const Eigen::Matrix2d& covs, const double& theta_cov, const geometry_msgs::Point& position, const bool& loopClosure) const
{
  visualization_msgs::Marker new_marker;
  new_marker.header = header;
  new_marker.type = visualization_msgs::Marker::SPHERE;
  new_marker.action = visualization_msgs::Marker::ADD;
  new_marker.color = ( loopClosure ? loop_color_ : covariance_color_ );
  new_marker.ns = "/positions";
  new_marker.id = id;
  
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(covs);

  const Eigen::Vector2d& eigValues (eig.eigenvalues());
  const Eigen::Matrix2d& eigVectors (eig.eigenvectors());
  double angle = (atan2(eigVectors(1, 0), eigVectors(0, 0)));
  double lengthMajor = sqrt(eigValues[0]);
  double lengthMinor = sqrt(eigValues[1]);

  new_marker.scale.x = ( std::isnormal(lengthMajor) ? lengthMajor : 0);
  new_marker.scale.y = ( std::isnormal(lengthMinor) ? lengthMinor : 0);
  new_marker.scale.z = 2 * sqrt(theta_cov) + 0.001;
  new_marker.pose.position.x = position.x;
  new_marker.pose.position.y = position.y;
  new_marker.pose.position.z = position.z;
  new_marker.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
  
  return new_marker;
}

void Trajectory2MarkersAlgNode::change_current_marker(const Eigen::Matrix2d& covs, const double& theta_cov, const geometry_msgs::Point& position)
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(covs);

  const Eigen::Vector2d& eigValues (eig.eigenvalues());
  const Eigen::Matrix2d& eigVectors (eig.eigenvectors());
  double angle = (atan2(eigVectors(1, 0), eigVectors(0, 0)));
  double lengthMajor = sqrt(eigValues[0]);
  double lengthMinor = sqrt(eigValues[1]);

  current_marker_.scale.x = ( std::isnormal(lengthMajor) ? lengthMajor : 0);
  current_marker_.scale.y = ( std::isnormal(lengthMinor) ? lengthMinor : 0);
  current_marker_.scale.z = 2 * sqrt(theta_cov) + 0.001;
  current_marker_.pose.position.x = position.x;
  current_marker_.pose.position.y = position.y;
  current_marker_.pose.position.z = position.z;
  current_marker_.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
}

Eigen::Matrix2d Trajectory2MarkersAlgNode::get_ith_cov(const iri_poseslam::Trajectory& msg, const uint i) const
{
  return get_cov(msg.poses.at(i));
}

double Trajectory2MarkersAlgNode::get_ith_theta_cov(const iri_poseslam::Trajectory& msg, const uint i) const
{
  return msg.poses.at(i).pose.covariance.at(8);
}

Eigen::Matrix2d Trajectory2MarkersAlgNode::get_cov(const geometry_msgs::PoseWithCovarianceStamped& p) const
{
  Eigen::Matrix2d covs;
  covs(0, 0) = p.pose.covariance.at(0);
  covs(0, 1) = p.pose.covariance.at(1);
  covs(1, 0) = p.pose.covariance.at(3);
  covs(1, 1) = p.pose.covariance.at(4);
  return covs;
}

double Trajectory2MarkersAlgNode::get_theta_cov(const geometry_msgs::PoseWithCovarianceStamped& p) const
{
  return p.pose.covariance.at(8);
}

