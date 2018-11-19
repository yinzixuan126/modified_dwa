#include "sensors_2_link_alg_node.h"

Sensors2LinkAlgNode::Sensors2LinkAlgNode(void) :
algorithm_base::IriBaseAlgorithm<Sensors2LinkAlgorithm>()
{
    ROS_DEBUG("SENSORS 2 LINK: initializing...");
  //init class attributes if necessary
  int N_scans_discard, fusion_mode;
  public_node_handle_.param<bool>("online_mode", online_mode_, true);
  public_node_handle_.param<int>("fusion_mode", fusion_mode, 2);
  public_node_handle_.param<int>("N_scans_discard", N_scans_discard, 0);
  public_node_handle_.param<double>("ICP_covariance_correction_factor", ICP_covariance_correction_factor_, 100.0);
  public_node_handle_.param<std::string>("base_frame_id", base_frame_id_, "/teo/base_link");
  public_node_handle_.param<std::string>("laser_frame_id", laser_frame_id_, "/teo/front_laser");
  public_node_handle_.param<bool>("allow_slipping", allow_slipping_, false);
  fusion_mode_ = uint(fusion_mode);
  N_scans_discard_ = uint(N_scans_discard);
  ROS_DEBUG("SENSORS 2 LINK: Config updated");
  
  //this->loop_rate_ = 2;//in [Hz]
  
  // [init publishers]
  
  // [init subscribers]
  this->scan_subscriber_ = this->public_node_handle_.subscribe("scan", 1000, &Sensors2LinkAlgNode::scan_callback, this);
  this->odom_relative_subscriber_ = this->public_node_handle_.subscribe("odom_relative", 1000, &Sensors2LinkAlgNode::odom_relative_callback, this);
  this->cmd_vel_subscriber_ = this->public_node_handle_.subscribe("cmd_vel", 100, &Sensors2LinkAlgNode::cmd_vel_callback, this);
  
  // [init services]
  this->get_link_server_ = this->public_node_handle_.advertiseService("get_link", &Sensors2LinkAlgNode::get_linkCallback, this);
  
  // [init clients]
  get_relative_pose_client_ = this->public_node_handle_.serviceClient<iri_laser_icp::GetRelativePose>("get_relative_pose");
  
  // [init action servers]
  
  // [init action clients]
  
  // init variables
  odom_rel_       = MatrixXd::Zero(3,1);
  odom_rel_cov_   = MatrixXd::Zero(3,3);
  Jp_              = MatrixXd::Identity(3,3);
  Jd_              = MatrixXd::Identity(3,3);
  fusion_ready_   = false;
  new_laser_scan_ = false;
  prev_seq_        = 0;
  laser_scan_counter_ = 0;
  currently_stopped_ = false;
  stopped_since_last_odom_ = true;
  tf_ready_ = false;
  T_base_laser_.setIdentity();
  T_laser_base_.setIdentity();
  load_tf();
  ROS_DEBUG("SENSORS 2 LINK: initialized!");
}

Sensors2LinkAlgNode::~Sensors2LinkAlgNode(void)
{
  // [free dynamic memory]
}

void Sensors2LinkAlgNode::mainNodeThread(void)
{
  if (!tf_ready_)
    load_tf();

  // [fill msg structures]
  
  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */
void Sensors2LinkAlgNode::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_DEBUG("SENSORS 2 LINK: New laser scan received! time = %f", msg->header.stamp.toSec());

  // LOAD THE LASER SCAN
  if (laser_scan_counter_ >= N_scans_discard_)
  {
    last_laser_scan_ = (*msg);
    new_laser_scan_ = true;
    laser_scan_counter_ = 0;
  }
  else
    laser_scan_counter_ += 1;

  // ADD FIRST SCAN DIRECTLY TO laser_scan_buffer_
  if (laser_scan_buffer_.empty())
  {
    laser_scan_buffer_.push_back(*msg);
    new_laser_scan_ = false; // avoid self pose odometry
    laser_scan_counter_ = 0;
  }
}

void Sensors2LinkAlgNode::odom_relative_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ROS_DEBUG("SENSORS 2 LINK: Odom_relative received! time = %f - seq = %i", msg->header.stamp.toSec(), msg->header.seq);

  if (prev_seq_ !=0 && prev_seq_ + 1 < msg->header.seq)
    ROS_WARN("SENSORS 2 LINK: Any odom relative lost! prev_seq_ = %i current_seq = %i", prev_seq_, msg->header.seq);

  prev_seq_ = msg->header.seq;

  Matrix3d Q = MatrixXd::Zero(3,3);
  d_local_ = MatrixXd::Zero(3,1);

  // Update the boolean stopped since last odom
  if (currently_stopped_)
  {
    if (!stopped_since_last_odom_)
      ROS_INFO("SENSORS 2 LINK: Robot has stopped");
    stopped_since_last_odom_ = true;
  }
  else
    stopped_since_last_odom_ = false;

  // If not stopped
  if (!stopped_since_last_odom_ || allow_slipping_)
  {
    // Odometry
    d_local_ = pose_2_vector(msg->pose.pose);

    // Odometry noise
    Q = covariance_2_matrix(msg->pose);


  }
  else
    ROS_DEBUG("SENSORS 2 LINK: Zero odometry (STOPPED)");
  
  // accumulate odometry
  // TIME
  odom_rel_time_ = msg->header.stamp;

  // COVARIANCE PROPAGATION
  // Reference Point Jacobian (previous orientation)
  Jp_(0,2) =-sin(odom_rel_(2)) * d_local_(0) - cos(odom_rel_(2)) * d_local_(1);
  Jp_(1,2) = cos(odom_rel_(2)) * d_local_(0) - sin(odom_rel_(2)) * d_local_(1);

  // Displacement Jacobian
  Jd_ = rotation_matrix(odom_rel_(2));

  // Covariance propagation
  odom_rel_cov_ = Jp_ * odom_rel_cov_ * Jp_.transpose() + Jd_ * Q * Jd_.transpose();

  if ((odom_rel_cov_.array() > 10).any())
  {
      ROS_ERROR("SENSORS 2 LINK: Too large odometry covariance after integration!");
      std::cout << "odom_rel_cov_" << std::endl << odom_rel_cov_ << std::endl;
      std::cout << "Q" << std::endl << Q << std::endl;
  }

  // POSE
  odom_rel_ += rotation_matrix(odom_rel_(2)) * d_local_;

  // ADD IN THE BUFFER
  // reserve space if needed
  if (odom_rel_buffer_.size() == odom_rel_buffer_.capacity())
  {
    odom_rel_time_buffer_.reserve(int(odom_rel_time_buffer_.size()) + 1000);
    odom_rel_cov_buffer_.reserve(int(odom_rel_cov_buffer_.size()) + 1000);
    odom_rel_buffer_.reserve(int(odom_rel_buffer_.size()) + 1000);
  }
  odom_rel_time_buffer_.push_back(odom_rel_time_);
  odom_rel_cov_buffer_.push_back(odom_rel_cov_);
  odom_rel_buffer_.push_back(odom_rel_);

  
  // ODOMETRY FUSION ISSUES

  // OFFLINE MODE
  if (!online_mode_ && odom_rel_buffer_.size() > 1 && new_laser_scan_ && last_laser_scan_.header.stamp < msg->header.stamp)
  {
    // reserve space if needed
    if (odom_buffer_.size() == odom_buffer_.capacity())
      odom_buffer_.reserve(int(odom_buffer_.size()) + 1000);
    if (laser_scan_buffer_.size() == laser_scan_buffer_.capacity())
      laser_scan_buffer_.reserve(int(laser_scan_buffer_.size()) + 1000);

    // ODOMETRY FUSION (LASER & ODOMETRY_RELATIVE)
    geometry_msgs::PoseWithCovarianceStamped new_odometry = odometry_fusion(last_laser_scan_, odom_rel_buffer_.size() - 1);
    laser_scan_buffer_.push_back(last_laser_scan_);
    odom_buffer_.push_back(new_odometry);
    new_laser_scan_ = false;

    ROS_DEBUG("SENSORS 2 LINK: New precomputed odometry stored!");
  }

  // ONLINE MODE
  else if (odom_rel_buffer_.size() > 1 && new_laser_scan_ && last_laser_scan_.header.stamp < msg->header.stamp)
  {
    ROS_DEBUG("SENSORS 2 LINK: Odom + scan fusion ready!");
    fusion_ready_ = true;
    new_laser_scan_ = false;
    ready_laser_scan_ = last_laser_scan_;
    ready_odom_id_ = odom_rel_buffer_.size() - 1;
  }
}

void Sensors2LinkAlgNode::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  ROS_DEBUG("SENSORS 2 LINK: cmd_vel callback: %f, %f, %f", msg->linear.x, msg->linear.y, msg->linear.z);
  currently_stopped_ = (msg->linear.x == 0 && msg->linear.y == 0 && msg->linear.z == 0 && msg->angular.x == 0 && msg->angular.y == 0 && msg->angular.z == 0);
  
  if (stopped_since_last_odom_ && !currently_stopped_)
    ROS_INFO("SENSORS 2 LINK: Robot has started moving");
  
  stopped_since_last_odom_ = stopped_since_last_odom_ && currently_stopped_;
}

/*  [service callbacks] */
bool Sensors2LinkAlgNode::get_linkCallback(iri_poseslam::GetLink::Request &req, iri_poseslam::GetLink::Response &res)
{
  ROS_DEBUG("SENSORS 2 LINK: New GetLink Request Received!");

  res.success = false;
  res.end = false;

  // FIRST CALLBACK (ask for first laser scan header)
  if (req.current_step == 0 && req.with_step == 0 )
  {
    ROS_DEBUG("SENSORS 2 LINK: first laser scan header Request Received!");
    if (!laser_scan_buffer_.empty())
    {
      res.odom.header = laser_scan_buffer_.front().header;
      res.odom.pose = eigen_2_posewithcovariance(VectorXd::Zero(3), MatrixXd::Zero(3,3));
      res.success = true;
      ROS_DEBUG("SENSORS 2 LINK: first laser scan header given");
    }
  }

  // NORMAL CALLBACK
  else
  {
    // ODOMETRY LINK
    if (req.current_step < req.with_step)
    {
      res = (online_mode_ ? online_odometry() : offline_odometry());
      if (res.success && !res.end && !is_semiPD(covariance_2_matrix(res.odom.pose))) 
      {
        res.success = false;
        std::cout << "ERROR: Odometry covariance is not PD!" << std::endl << covariance_2_matrix(res.odom.pose) << std::endl;
      }
    }

    // LOOP CLOSURE LINK
    else
    {  
      get_relative_pose_srv_.request.scan_sens = laser_scan_buffer_.at(req.current_step);
      get_relative_pose_srv_.request.scan_ref = laser_scan_buffer_.at(req.with_step);
      get_relative_pose_srv_.request.prior_d = base_2_laser_frame(req.prior_d); // CHANGE FRAME TO LASER FRAME

      ROS_DEBUG("SENSORS 2 LINK: ICP in Loop closure stamps:\n%i.%i\n%i %i", 
                          laser_scan_buffer_.at(req.current_step).header.stamp.sec, 
                          laser_scan_buffer_.at(req.current_step).header.stamp.nsec, 
                          laser_scan_buffer_.at(req.with_step).header.stamp.sec, 
                          laser_scan_buffer_.at(req.with_step).header.stamp.nsec);

      // call ICP
      if (get_relative_pose_client_.call(get_relative_pose_srv_) && get_relative_pose_srv_.response.success)
      {
        res.odom.header = get_relative_pose_srv_.request.scan_sens.header;
        res.odom.pose = laser_2_base_frame(get_relative_pose_srv_.response.pose_rel.pose); // CHANGE FRAME TO BASE FOOTPRINT FRAME
        res.success = true;
        ROS_DEBUG("SENSORS 2 LINK: ICP in Loop closure\nPS  prior  = %f, %f, %f\nICP prior  = %f, %f, %f\nICP result = %f, %f, %f\nPS  result = %f, %f, %f", 
                            req.prior_d.position.x, req.prior_d.position.y, tf::getYaw(req.prior_d.orientation), 
                            get_relative_pose_srv_.request.prior_d.position.x, get_relative_pose_srv_.request.prior_d.position.y, tf::getYaw(get_relative_pose_srv_.request.prior_d.orientation),
                            get_relative_pose_srv_.response.pose_rel.pose.pose.position.x, get_relative_pose_srv_.response.pose_rel.pose.pose.position.y, tf::getYaw(get_relative_pose_srv_.response.pose_rel.pose.pose.orientation),
                            res.odom.pose.pose.position.x, res.odom.pose.pose.position.y, tf::getYaw(res.odom.pose.pose.orientation));

        if (!is_semiPD(covariance_2_matrix(res.odom.pose)))
        {
            ROS_ERROR("SENSORS 2 LINK: Loop closure ICP_cov is not PD!");
            std::cout << covariance_2_matrix(res.odom.pose) << std::endl;
            res.success = false;
        }
      }
      else
      {
        ROS_ERROR("SENSORS 2 LINK: ICP communication failed or didn't found a matchin loop closure");
        res.success = false;
      }
    }
  }
  return true;
}

/*  [action callbacks] */

/*  [action requests] */

void Sensors2LinkAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->alg_.unlock();
}

void Sensors2LinkAlgNode::addNodeDiagnostics(void)
{
}

iri_poseslam::GetLink::Response Sensors2LinkAlgNode::offline_odometry()
{
  iri_poseslam::GetLink::Response result;
 
  // Return the pre-computed odometry
  if (!odom_buffer_.empty())
  {
    result.odom = odom_buffer_.front();
    result.success = true;
    result.end = (result.odom.pose.covariance.at(0) == 0 && result.odom.pose.covariance.at(7) == 0 && result.odom.pose.covariance.at(35) == 0 ? true : false);

    // Erase from the buffer
    odom_buffer_.erase(odom_buffer_.begin());
  }
  else
  {
    //ROS_WARN("SENSORS 2 LINK: There is no more precomputed odometries");
    result.success = false;
    result.end = true;
  }
  
  return result;
}

iri_poseslam::GetLink::Response Sensors2LinkAlgNode::online_odometry()
{
  //ROS_INFO("SENSORS 2 LINK: Online Odometry");
  iri_poseslam::GetLink::Response result;
  
  if (fusion_ready_)
  {
    // Call sensor fusion
    result.odom = odometry_fusion(ready_laser_scan_, ready_odom_id_);
    result.success = true;
    // zero motion check
    if (result.odom.pose.covariance.at(0) != 0 || result.odom.pose.covariance.at(7) != 0 || result.odom.pose.covariance.at(35) != 0)
    {
      result.end = false;
      // reserve space if needed
      if (laser_scan_buffer_.size() == laser_scan_buffer_.capacity())
        laser_scan_buffer_.reserve(int(laser_scan_buffer_.size()) + 1000);
      // Add laser scan in the laser_scan_buffer_
      laser_scan_buffer_.push_back(ready_laser_scan_);
    }
    else
      result.end = true;

    ROS_DEBUG("SENSORS 2 LINK: New odometry stamp:\n%i.%i", 
                    result.odom.header.stamp.sec, 
                    result.odom.header.stamp.nsec);
  }
  else
  {
    ROS_DEBUG("SENSORS 2 LINK: There is not enough sensor data");
    result.success = false;
    result.end = true;
  }
  
  return result;
}

geometry_msgs::PoseWithCovarianceStamped Sensors2LinkAlgNode::odometry_fusion(const sensor_msgs::LaserScan &laser_scan, const int &odom_rel_idx)
{
  Vector3d odom, odom_ICP, int_odom_rel;
  Matrix3d odom_cov, odom_ICP_cov, int_odom_rel_cov;
  
  // RELATIVE ODOMETRY INTERPOLATION
  interpolate_odom_rel(odom_rel_idx, laser_scan.header.stamp, int_odom_rel, int_odom_rel_cov);

  // Motion zero
  if (int_odom_rel_cov.isZero())
  {
    ROS_DEBUG("SENSORS 2 LINK: Zero motion odometry");
    odom_cov = Matrix3d::Zero();
    odom = Vector3d::Zero();
  }
  // Fusion mode 3: Encoder + IMU odometry
  else if (fusion_mode_ == 3)
  {
    odom = int_odom_rel;
    odom_cov = int_odom_rel_cov;
  }
  else
  {
    // LASER SCANS ICP
    get_relative_pose_srv_.request.scan_sens = laser_scan;  // next laser scan
    get_relative_pose_srv_.request.scan_ref = laser_scan_buffer_.back(); // current laser scan
    get_relative_pose_srv_.request.prior_d = base_2_laser_frame(vector_2_pose(int_odom_rel));
    
    // call ICP
    if (get_relative_pose_client_.call(get_relative_pose_srv_) && get_relative_pose_srv_.response.success)
    {
      // CHANGE FRAME TO BASE FOOTPRINT
      laser_2_base_frame(get_relative_pose_srv_.response.pose_rel.pose, odom_ICP, odom_ICP_cov); 

      ROS_DEBUG("SENSORS 2 LINK: ICP in odometry\nPS  prior  = %f, %f, %f\nICP prior  = %f, %f, %f\nICP result = %f, %f, %f\nPS  result = %f, %f, %f", 
                 int_odom_rel(0), int_odom_rel(1), int_odom_rel(2), 
                 get_relative_pose_srv_.request.prior_d.position.x, get_relative_pose_srv_.request.prior_d.position.y, tf::getYaw(get_relative_pose_srv_.request.prior_d.orientation),
                 get_relative_pose_srv_.response.pose_rel.pose.pose.position.x, get_relative_pose_srv_.response.pose_rel.pose.pose.position.y, tf::getYaw(get_relative_pose_srv_.response.pose_rel.pose.pose.orientation),
                 odom_ICP(0), odom_ICP(1), odom_ICP(2));
      

      // FUSION
      switch (fusion_mode_)
      {
        case 1: // Fusion mode 1: ODOM + ICP(ODOM)
          odom_cov = (int_odom_rel_cov.inverse() + odom_ICP_cov.inverse()).inverse();
          odom = odom_cov * (int_odom_rel_cov.inverse() * int_odom_rel + odom_ICP_cov.inverse() * odom_ICP);
          break;

        case 2: // Fusion mode 2: ICP(ODOM)
          odom = odom_ICP;
          odom_cov = odom_ICP_cov;
          break;

        default: 
          ROS_ERROR("SENSORS 2 LINK: Invalid fusion mode %i. ICP odometry selected.",fusion_mode_);
          fusion_mode_ = 2;
          odom = odom_ICP;
          odom_cov = odom_ICP_cov;
          break;
      }
    
      // ICP OUTLIER DETECTION
      // Mahalanobis distance (from odometry reading)
      double md = sqrt((odom_ICP - int_odom_rel).transpose() * (100 * int_odom_rel_cov).inverse() * (odom_ICP - int_odom_rel));

      if (md > alg_.K_mahalanobis_)
      {
        odom = int_odom_rel;
        odom_cov = int_odom_rel_cov;
        ROS_DEBUG("SENSORS 2 LINK: ICP outlier detected in odometry! Changed to encoders+IMU odometry");
      }
      else if (!is_semiPD(odom_ICP_cov)) 
      {
        odom = int_odom_rel;
        odom_cov = int_odom_rel_cov;
        ROS_DEBUG("SENSORS 2 LINK: ICP transformed covariance is not PD! Changed to encoders+IMU odometry");
      }
      else if (100 * int_odom_rel_cov.determinant() < odom_cov.determinant() || 100 * int_odom_rel_cov.maxCoeff() < odom_cov.maxCoeff() || odom_cov.maxCoeff() > 1e2)
      {
        odom = int_odom_rel;
        odom_cov = int_odom_rel_cov;
        ROS_DEBUG("SENSORS 2 LINK: Huge covariance in odometry! Changed to encoders+IMU odometry");
      }
      //ROS_INFO("odom_ICP_cov:");
      //ROS_INFO_STREAM(odom_ICP_cov);
      //ROS_INFO("int_odom_rel_cov:");
      //ROS_INFO_STREAM(int_odom_rel_cov);
      //ROS_INFO("odom_cov:");
      //ROS_INFO_STREAM(odom_cov);
      //ROS_INFO("SENSORS 2 LINK: Fused odometry %f %f %f", odom(0), odom(1), odom(2));
    }
    else
    {
      ROS_WARN("SENSORS 2 LINK: ICP communication failed or ICP didn't found a match in odometry fusion. Changed to encoders+IMU odometry");
      odom_cov = int_odom_rel_cov;
      odom = int_odom_rel;
    }
  }

  //ROS_INFO("SENSORS 2 LINK\nlaser_scan_buffer_.size() = %lu\nLaserScan stamp: %i.%i", laser_scan_buffer_.size(), laser_scan.header.stamp.sec,laser_scan.header.stamp.nsec);
  
  // TRANSLATING to a PoseWithCovariancestamped
  geometry_msgs::PoseWithCovarianceStamped fused_odom;
  fused_odom.header = laser_scan.header;
  fused_odom.pose = eigen_2_posewithcovariance(odom, odom_cov);
  
  fusion_ready_ = false;

  return fused_odom;
}

void Sensors2LinkAlgNode::interpolate_odom_rel(const int &odom_rel_idx, const ros::Time scan_stamp, Vector3d& int_odom_rel, Matrix3d& int_odom_rel_cov)
{
  Vector3d pre_odom_rel  = odom_rel_buffer_.at(odom_rel_idx - 1);
  Vector3d post_odom_rel = odom_rel_buffer_.at(odom_rel_idx);
  Matrix3d pre_odom_rel_cov  = odom_rel_cov_buffer_.at(odom_rel_idx - 1);
  Matrix3d post_odom_rel_cov = odom_rel_cov_buffer_.at(odom_rel_idx);
  ros::Time pre_odom_rel_time  = odom_rel_time_buffer_.at(odom_rel_idx - 1);
  ros::Time post_odom_rel_time = odom_rel_time_buffer_.at(odom_rel_idx);

  // Interpolation
  double alpha;
  if (scan_stamp < pre_odom_rel_time || post_odom_rel_time == pre_odom_rel_time)
    alpha = 0;
  else
  //  alpha = (scan_stamp - pre_odom_rel_time).toSec() / (post_odom_rel_time - pre_odom_rel_time).toSec();
    alpha = 1; // NOT INTERPOLE: posterior odometry message
  //alpha = 0; // NOT INTERPOLE: previous odometry message
  int_odom_rel = (1 - alpha) * pre_odom_rel + alpha * post_odom_rel;
  int_odom_rel_cov = (1 - alpha) * pre_odom_rel_cov + alpha * post_odom_rel_cov;

  // Update relative odometries buffers
  MatrixXd rot = rotation_matrix(int_odom_rel(2));
  VectorXd rest_odom_rel = rot * (post_odom_rel - int_odom_rel);

  rot = rotation_matrix(-int_odom_rel(2));
  MatrixXd rest_odom_rel_cov = rot * (post_odom_rel_cov - int_odom_rel_cov) * rot.transpose();

  odom_rel_buffer_.clear();
  odom_rel_cov_buffer_.clear();
  odom_rel_time_buffer_.clear();

  odom_rel_buffer_.push_back(rest_odom_rel);
  odom_rel_cov_buffer_.push_back(rest_odom_rel_cov);
  odom_rel_time_buffer_.push_back(pre_odom_rel_time + (post_odom_rel_time - pre_odom_rel_time) * alpha);

  // initialize odom relative
  odom_rel_ = rest_odom_rel; //MatrixXd::Zero(3,1);
  odom_rel_cov_ = rest_odom_rel_cov; //MatrixXd::Zero(3,3);
}

geometry_msgs::Pose Sensors2LinkAlgNode::base_2_laser_frame(const geometry_msgs::Pose &base_disp)
{
  tf::Transform T_base_disp, T_laser_disp;
  tf::poseMsgToTF(base_disp, T_base_disp);
  T_laser_disp = T_laser_base_ * T_base_disp * T_base_laser_;
  geometry_msgs::Pose laser_disp;
  tf::poseTFToMsg(T_laser_disp, laser_disp);
  
  return laser_disp;
}

geometry_msgs::PoseWithCovariance Sensors2LinkAlgNode::laser_2_base_frame(const geometry_msgs::PoseWithCovariance &laser_disp_cov)
{
  geometry_msgs::PoseWithCovariance base_disp_cov;

  tf::Transform T_base_disp, T_laser_disp;
  tf::poseMsgToTF(laser_disp_cov.pose, T_laser_disp);
  T_base_disp = T_base_laser_ * T_laser_disp * T_laser_base_;
  tf::poseTFToMsg(T_base_disp, base_disp_cov.pose);
  
  Matrix3d J = rotation_matrix(tf::getYaw(base_disp_cov.pose.orientation));
  Matrix3d odom_ICP_cov = (J * covariance_2_matrix(laser_disp_cov) * J.transpose()) * ICP_covariance_correction_factor_; // COVARIANCE CORRECTION
  base_disp_cov.covariance = matrix_2_covariance(odom_ICP_cov);

  return base_disp_cov;
}

void Sensors2LinkAlgNode::laser_2_base_frame(const geometry_msgs::PoseWithCovariance &laser_disp_cov, Vector3d &odom_ICP, Matrix3d &odom_ICP_cov)
{
  tf::Transform T_base_disp, T_laser_disp;
  tf::poseMsgToTF(laser_disp_cov.pose, T_laser_disp);
  T_base_disp = T_base_laser_ * T_laser_disp * T_laser_base_;
  geometry_msgs::Pose base_disp;
  tf::poseTFToMsg(T_base_disp, base_disp);
  
  odom_ICP = pose_2_vector(base_disp);
  Matrix3d J = rotation_matrix(odom_ICP(2));
  odom_ICP_cov = (J * covariance_2_matrix(laser_disp_cov) * J.transpose()) * ICP_covariance_correction_factor_; // COVARIANCE CORRECTION
}

Matrix3d Sensors2LinkAlgNode::rotation_matrix(const double &alpha) const
{
  Matrix3d rot = MatrixXd::Identity(3,3);
  
  rot(0,0) = cos(alpha);
  rot(0,1) = -sin(alpha);
  rot(1,0) = sin(alpha);
  rot(1,1) = cos(alpha);

  return rot;
}

// CONVERSION FUNCTIONS (EIGEN - ROS_MSGS)

Matrix3d Sensors2LinkAlgNode::covariance_2_matrix(const geometry_msgs::PoseWithCovariance &pose) const
{
  Matrix3d cov;
  
  cov(0,0) = pose.covariance.at(0);
  cov(0,1) = pose.covariance.at(1);
  cov(0,2) = pose.covariance.at(5);
  cov(1,0) = pose.covariance.at(6);
  cov(1,1) = pose.covariance.at(7);
  cov(1,2) = pose.covariance.at(11);
  cov(2,0) = pose.covariance.at(30);
  cov(2,1) = pose.covariance.at(31);
  cov(2,2) = pose.covariance.at(35);

  return cov;
}

Vector3d Sensors2LinkAlgNode::pose_2_vector(const geometry_msgs::Pose &pose) const
{
  Vector3d p;
  
  p(0) = pose.position.x;
  p(1) = pose.position.y;
  p(2) = tf::getYaw(pose.orientation);
  
  return p;
}

geometry_msgs::PoseWithCovariance Sensors2LinkAlgNode::eigen_2_posewithcovariance(const Vector3d &p, const Matrix3d &cov) const
{
  geometry_msgs::PoseWithCovariance pose;

  pose.pose = vector_2_pose(p);
  
  pose.covariance.at(0)  = cov(0,0);
  pose.covariance.at(1)  = cov(0,1);
  pose.covariance.at(5)  = cov(0,2);
  pose.covariance.at(6)  = cov(1,0);
  pose.covariance.at(7)  = cov(1,1);
  pose.covariance.at(11) = cov(1,2);
  pose.covariance.at(30) = cov(2,0);
  pose.covariance.at(31) = cov(2,1);
  pose.covariance.at(35) = cov(2,2);
  
  return pose;
}
boost::array<double, 36> Sensors2LinkAlgNode::matrix_2_covariance(const Matrix3d &cov) const
{
  boost::array<double, 36> covariance;

  covariance.at(0)  = cov(0,0);
  covariance.at(1)  = cov(0,1);
  covariance.at(5)  = cov(0,2);
  covariance.at(6)  = cov(1,0);
  covariance.at(7)  = cov(1,1);
  covariance.at(11) = cov(1,2);
  covariance.at(30) = cov(2,0);
  covariance.at(31) = cov(2,1);
  covariance.at(35) = cov(2,2);

  return covariance;
}

geometry_msgs::Pose Sensors2LinkAlgNode::vector_2_pose(const Vector3d &p) const
{
  geometry_msgs::Pose pose;

  pose.position.x = p(0);
  pose.position.y = p(1);
  pose.position.z = 0;
  pose.orientation = tf::createQuaternionMsgFromYaw(p(2));
  
  return pose;
}

void Sensors2LinkAlgNode::load_tf()
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
    ROS_WARN("SENSORS 2 LINK: Transform exception: %s", ex.what());
  }
}

// Check if matrix M is Semi Positive Definite
bool Sensors2LinkAlgNode::is_semiPD(const Matrix3d& M) const
{
  EigenSolver<Matrix3d> es(M);
  //VectorXcd eigvals = M.eigenvalues();

  if ((es.eigenvalues().real().array() < 0).any() || !es.eigenvalues().imag().isZero())
  {
    std::cout << "Not Semipositive definite!"<< std::endl << es.eigenvalues() << std::endl;
    // MatrixXd EIGS = MatrixXd((es.eigenvalues().real().array() >= 0).select(es.eigenvalues().real(),0).asDiagonal()); //MatrixXd(es.eigenvalues().real().array().abs().matrix().asDiagonal());
    // std::cout << "New forced eigenvalues"<< std::endl << EIGS << std::endl;
    // std::cout << "Original matrix"<< std::endl << M << std::endl;
    // M = es.eigenvectors().real() * EIGS * es.eigenvectors().real().inverse();
    // std::cout << "Semidefinite positive forced matrix"<< std::endl << M << std::endl;
    // std::cout << "New matrix eigenvalues"<< std::endl << M.eigenvalues() << std::endl;

    return false;
  }
  else
    return true;
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<Sensors2LinkAlgNode>(argc, argv, "sensors_2_link_node");
}
