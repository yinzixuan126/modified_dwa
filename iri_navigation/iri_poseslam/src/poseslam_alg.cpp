#include "poseslam_alg.h"

PoseslamAlgorithm::PoseslamAlgorithm(void)
{ 
  pthread_mutex_init(&this->access_,NULL);
  Q_odom_ = MatrixXd::Zero(3,3);
}

PoseslamAlgorithm::~PoseslamAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
  delete pose_SLAM_;
}

void PoseslamAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  Params Parameters = pose_SLAM_->get_params();
  Parameters.matchArea(0) = new_cfg.match_area_x;
  Parameters.matchArea(1) = new_cfg.match_area_y;
  Parameters.matchArea(2) = new_cfg.match_area_th;
  Parameters.pdRange.first = new_cfg.pd_range_1;   // Probability threshold of 2 poses of being closer than 'matchArea' for trying to create a loop.
  Parameters.pdRange.second = new_cfg.pd_range_2;  // Probability threshold of 2 poses of being closer than 'matchArea' for one of them to be redundant.
  Parameters.igRange.first = new_cfg.ig_range_1;   // Information gain threshold for a pose to be redundant 
  Parameters.igRange.second = new_cfg.ig_range_2;  // Information gain threshold for a try loop closure
  Parameters.ignorePrevious = new_cfg.ignore_previous_steps; // Number of previous states to ignore on loop closure
  Parameters.K_mahalanobis = new_cfg.K_mahalanobis; // Mahalanobis distance in the outlier detection in loop closure
  pose_SLAM_->set_parameters(Parameters);
  
  this->unlock();
}

void PoseslamAlgorithm::initialize(const Vector3d& init_mu, const Matrix3d& init_S, const Params& Parameters)
{
  ROS_INFO("\nPOSE SLAM: Creating PoseSLAM");
  pose_SLAM_ = new CPoseSLAM(init_mu, init_S, Parameters);

  ROS_INFO("\nPOSE SLAM:\n------------ INITIALIZED -------------\nInit pose:\n\t[%f, %f %f]\nInit cov:\n\t[%f, %f, %f]\n\t[%f, %f, %f]\n\t[%f, %f, %f]\n",
                    init_mu(0), init_mu(1), init_mu(2),
                    init_S(0, 0),init_S(0, 1),init_S(0, 2),
                    init_S(1, 0),init_S(1, 1),init_S(1, 2),
                    init_S(2, 0),init_S(2, 1),init_S(2, 2));
}

// PoseslamAlgorithm Public API
void PoseslamAlgorithm::augmentation(const uint& step, const geometry_msgs::PoseWithCovarianceStamped& odom)
{  

  Q_odom_(0,0) = odom.pose.covariance.at(0);
  Q_odom_(0,1) = odom.pose.covariance.at(1);
  Q_odom_(0,2) = odom.pose.covariance.at(5);
  Q_odom_(1,0) = odom.pose.covariance.at(6);
  Q_odom_(1,1) = odom.pose.covariance.at(7);
  Q_odom_(1,2) = odom.pose.covariance.at(11);
  Q_odom_(2,0) = odom.pose.covariance.at(30);
  Q_odom_(2,1) = odom.pose.covariance.at(31);
  Q_odom_(2,2) = odom.pose.covariance.at(35);
  
  Vector3d d;
  d(0) = odom.pose.pose.position.x;
  d(1) = odom.pose.pose.position.y;
  d(2) = tf::getYaw(odom.pose.pose.orientation);
  
  Q_aug_.push_back(Q_odom_);
  d_aug_.push_back(d);
  
  pose_SLAM_->augmentation(step, d, Q_odom_);

  // Zero motion link
  if (d.isZero() && Q_odom_.isZero())
  {
    pose_SLAM_->set_redundant(true);
    ROS_DEBUG("PS: Zero Motion -> Redundant pose");
  }
}

void PoseslamAlgorithm::create_candidates_list()
{
  pose_SLAM_->create_candidates_list();
}

void PoseslamAlgorithm::redundant_evaluation()
{
  pose_SLAM_->redundant_evaluation();
}

bool PoseslamAlgorithm::loop_closure_requeriments() const
{
  return pose_SLAM_->loop_closure_requeriments();
}

bool PoseslamAlgorithm::try_loop_closure(const geometry_msgs::PoseWithCovarianceStamped& odom)
{
  MatrixXd Q = MatrixXd::Zero(3, 3);

  Q(0,0) = odom.pose.covariance.at(0);
  Q(0,1) = odom.pose.covariance.at(1);
  Q(0,2) = odom.pose.covariance.at(5);
  Q(1,0) = odom.pose.covariance.at(6);
  Q(1,1) = odom.pose.covariance.at(7);
  Q(1,2) = odom.pose.covariance.at(11);
  Q(2,0) = odom.pose.covariance.at(30);
  Q(2,1) = odom.pose.covariance.at(31);
  Q(2,2) = odom.pose.covariance.at(35);
  
  Vector3d d;
  d(0) = odom.pose.pose.position.x;
  d(1) = odom.pose.pose.position.y;
  d(2) = tf::getYaw(odom.pose.pose.orientation);
  
  Q_loop_.push_back(Q);
  d_loop_.push_back(d);
  
  bool success = pose_SLAM_->try_loop_closure(Q, d);
  
  success_loop_.push_back(success);
    
  return success;
}

void PoseslamAlgorithm::update_candidates_list()
{
  pose_SLAM_->update_candidates_list();
}

bool PoseslamAlgorithm::any_candidate() const
{
  return pose_SLAM_->any_candidate(); 
}

void PoseslamAlgorithm::select_best_candidate()
{
  pose_SLAM_->select_best_candidate();
}

uint PoseslamAlgorithm::get_candidate_step() const
{
  return pose_SLAM_->get_candidate_step();
}

geometry_msgs::Pose PoseslamAlgorithm::get_candidate_d() const
{
  geometry_msgs::Pose disp;
  VectorXd d = pose_SLAM_->get_candidate_link().d;
  
  disp.position.x = d(0);
  disp.position.y = d(1);
  disp.position.z = 0;
  
  disp.orientation = tf::createQuaternionMsgFromYaw(d(2));
  
  return disp;
}

double PoseslamAlgorithm::get_candidate_ig() const
{
  return pose_SLAM_->get_candidate_link().iGain;
}

bool PoseslamAlgorithm::is_redundant() const
{
  return pose_SLAM_->is_redundant();
}

std::vector<VectorXd> PoseslamAlgorithm::get_trajectory() const
 {
  return pose_SLAM_->get_trajectory();
}   
   
std::vector<std::vector<double> > PoseslamAlgorithm::get_trajectory_covariance() const
{
  std::vector<MatrixXd> covs = pose_SLAM_->get_trajectory_covariance();
  std::vector<std::vector<double> > trajectory_covariances(covs.size());
  
  uint cov_size = covs.at(0).size();
    
  for (uint k = 0; k < covs.size(); k++)
  {
    std::vector<double> covariance(cov_size);
  
    std::copy(covs.at(k).data(),covs.at(k).data() + cov_size, covariance.begin());
    trajectory_covariances.at(k) = covariance;
  }
  return trajectory_covariances;
}

std::vector<uint> PoseslamAlgorithm::get_trajectory_steps() const
{
  return pose_SLAM_->get_trajectory_steps();
}

VectorXd PoseslamAlgorithm::get_last_pose() const
{
  return pose_SLAM_-> get_FF().get_PD().back().get_mean().get_vector();
}

std::vector<double> PoseslamAlgorithm::get_last_covariance() const
{
  MatrixXd last_covariance_matrix = pose_SLAM_-> get_FF().get_PD().back().get_S();
  std::vector<double> last_covariance(last_covariance_matrix.size());
  
  std::copy(last_covariance_matrix.data(), last_covariance_matrix.data() + last_covariance_matrix.size(), last_covariance.begin());
  return last_covariance;
}

uint PoseslamAlgorithm::get_last_step() const
{  
  return pose_SLAM_-> get_FF().get_PF().state_2_step(pose_SLAM_-> get_FF().get_nStates() - 1);
}

uint PoseslamAlgorithm::get_nStates() const
{
  return pose_SLAM_-> get_FF().get_nStates();
}