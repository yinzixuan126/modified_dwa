#include "poseslam_alg_node.h"

PoseslamAlgNode::PoseslamAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<PoseslamAlgorithm>(),
  step_(0)
{
  //init class attributes if necessary
  double lr;
  public_node_handle_.param<double>("loop_rate", lr, 20); //in [Hz]
  loop_rate_ = lr;
  
  // Parameters 
  Params Parameters;
  Parameters.LoopNoise = MatrixXd::Zero(3,3);
  public_node_handle_.param<double>("closing_loop_noise_xx", Parameters.LoopNoise(0, 0), 0.05);
  public_node_handle_.param<double>("closing_loop_noise_xy", Parameters.LoopNoise(0, 1), 0.0);
  public_node_handle_.param<double>("closing_loop_noise_xth", Parameters.LoopNoise(0, 2), 0.00);
  public_node_handle_.param<double>("closing_loop_noise_xy", Parameters.LoopNoise(1, 0), 0.0);
  public_node_handle_.param<double>("closing_loop_noise_yy", Parameters.LoopNoise(1, 1), 0.05);
  public_node_handle_.param<double>("closing_loop_noise_yth", Parameters.LoopNoise(1, 2), 0.0);
  public_node_handle_.param<double>("closing_loop_noise_xth", Parameters.LoopNoise(2, 0), 0.0);
  public_node_handle_.param<double>("closing_loop_noise_yth", Parameters.LoopNoise(2, 1), 0.0);
  public_node_handle_.param<double>("closing_loop_noise_thth", Parameters.LoopNoise(2, 2), 0.0005);
  public_node_handle_.param<double>("min_cov", Parameters.min_cov, 1e-8);
  // Initial position
  Vector3d init_mu;
  public_node_handle_.param<double>("initial_position_x", init_mu(0), 0.0);
  public_node_handle_.param<double>("initial_position_y", init_mu(1), 0.0);
  public_node_handle_.param<double>("initial_position_th", init_mu(2), 0.0);
  // Initial covariance
  MatrixXd init_S(3, 3);
  public_node_handle_.param<double>("initial_covariance_xx", init_S(0, 0), 1e-3);
  public_node_handle_.param<double>("initial_covariance_xy", init_S(0, 1), 0.0);
  public_node_handle_.param<double>("initial_covariance_xth", init_S(0, 2), 0.0);
  public_node_handle_.param<double>("initial_covariance_xy", init_S(1, 0), 0.0);
  public_node_handle_.param<double>("initial_covariance_yy", init_S(1, 1), 1e-3);
  public_node_handle_.param<double>("initial_covariance_yth", init_S(1, 2), 0.0);
  public_node_handle_.param<double>("initial_covariance_xth", init_S(2, 0), 0.0);
  public_node_handle_.param<double>("initial_covariance_yth", init_S(2, 1), 0.0);
  public_node_handle_.param<double>("initial_covariance_thth", init_S(2, 2), 1e-5);
  // Initialize filter
  alg_.initialize(init_mu, init_S, Parameters);

  // [init publishers]
  this->trajectory_publisher_ = this->public_node_handle_.advertise<iri_poseslam::Trajectory>("trajectory", 1);
  // [init subscribers]
  // [init services]
  // [init clients]
  get_link_client_ = this->public_node_handle_.serviceClient<iri_poseslam::GetLink>("get_link");
  // [init action servers]
  // [init action clients]
}

PoseslamAlgNode::~PoseslamAlgNode(void)
{
  // [free dynamic memory]
}

void PoseslamAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  // [fill srv structure and make request to the server]
  // [fill action structure and make request to the action server]
  // [publish messages]

  //ROS_INFO("POSE SLAM: step_ = %u",step_);

  // ---------- FIRST POSE -----------
  while (step_ == 0)
  {
    // First Laser Scan Header
    get_link_srv_.request.current_step = 0;
    get_link_srv_.request.with_step = 0;
    if (get_link_client_.call(get_link_srv_) && get_link_srv_.response.success)
    {
      augment_trajectories(get_link_srv_.response.odom.header);
      update_trajectories(false);

      this->trajectory_publisher_.publish(this->Trajectory_msg_);
      step_++;
    	
      ROS_DEBUG("POSE SLAM: FIRST STATE - step_: %i\npose: %f, %f, %f", step_, alg_.get_last_pose()(0), alg_.get_last_pose()(1), alg_.get_last_pose()(2));
    }
    else
      ROS_DEBUG("POSE SLAM: Couldn't get FIRST STATE");
  }
  
  // ---------- POSE SLAM ROUTINE -----------
  bool LoopClosure = false;
  // Odometry reception
  get_link_srv_.request.current_step = step_ - 1;
  get_link_srv_.request.with_step = step_;
  
  ROS_DEBUG("POSE SLAM: New pose odometry requested! step_ = %i - previous step_ = %i", step_, step_ - 1);
  if (get_link_client_.call(get_link_srv_))
  {
    if (get_link_srv_.response.success && !get_link_srv_.response.end)
    {    	
    	// State Augmentation
    	alg_.augmentation(step_, get_link_srv_.response.odom);
    	
      // Augment trajectory
      augment_trajectories(get_link_srv_.response.odom.header);
      ROS_DEBUG("POSE SLAM: New odometry\nnSteps = %lu\nstamp:\n%i.%i", 
                           Trajectory_msg_.poses.size(),
                           get_link_srv_.response.odom.header.stamp.sec, 
                           get_link_srv_.response.odom.header.stamp.nsec);

    	// Create list of link candidates
    	alg_.create_candidates_list();
    	
    	// Process all link candidates for loop closure if current pose has not been setted as redundant
    	while (alg_.any_candidate() && !alg_.is_redundant())
    	{
    	  // Select the most information gain link candidate
    	  alg_.select_best_candidate();
    	  
    	  // Check if the current pose is redundant
    	  alg_.redundant_evaluation();

        // Check if we can try loop closure
        if (alg_.loop_closure_requeriments())
    	  {
    	    get_link_srv_.request.current_step = step_;
    	    get_link_srv_.request.with_step = alg_.get_candidate_step();
    	    get_link_srv_.request.prior_d = alg_.get_candidate_d();
    	    
          ROS_DEBUG("POSE SLAM: trying to close a loop steps %lu-%lu\n%i.%i\n%i.%i", 
                               get_link_srv_.request.current_step, 
                               get_link_srv_.request.with_step,
                               Trajectory_msg_.poses.at(get_link_srv_.request.current_step).header.stamp.sec,
                               Trajectory_msg_.poses.at(get_link_srv_.request.current_step).header.stamp.nsec,
                               Trajectory_msg_.poses.at(get_link_srv_.request.with_step).header.stamp.sec,
                               Trajectory_msg_.poses.at(get_link_srv_.request.with_step).header.stamp.nsec);
    	    
    	    if (get_link_client_.call(get_link_srv_))
          {
            if (get_link_srv_.response.success && alg_.try_loop_closure(get_link_srv_.response.odom))
            {
              ROS_DEBUG("POSE SLAM: LOOP CLOSED! current step: %i with step: %i\nRelative distance: [%f, %f, %f]", step_, alg_.get_candidate_step(), get_link_srv_.response.odom.pose.pose.position.x, get_link_srv_.response.odom.pose.pose.position.y, tf::getYaw(get_link_srv_.response.odom.pose.pose.orientation));
              if (loops_.size() == loops_.capacity())
                loops_.reserve(loops_.size() * 100);
              loops_.push_back(alg_.get_candidate_step());
              loops_.push_back(step_);
              alg_.update_candidates_list();
              LoopClosure = true;
            }
          }
    	    else
    	      ROS_ERROR("POSE SLAM: Communication with sensors_2_link failed in loop closure");
    	  }
    	}
    	
    	if (!alg_.is_redundant()) 
    	  ROS_DEBUG("POSE SLAM: NEW STATE %i - step_: %i - IG: %f\npose: %f, %f, %f", alg_.get_nStates(), step_, alg_.get_candidate_ig(), alg_.get_last_pose()(0), alg_.get_last_pose()(1), alg_.get_last_pose()(2));
    	
    	// Trajectories publishing messages
    	update_trajectories(LoopClosure);
    	this->trajectory_publisher_.publish(this->Trajectory_msg_);
        ROS_DEBUG("PS: Trajectory published!");
    		
    	step_++;
    }
  }
  else 
    ROS_ERROR("POSE SLAM: Communication with sensors_2_link failed in next odom");
}

/*  [subscriber callbacks] */
/*  [service callbacks] */
/*  [action callbacks] */
/*  [action requests] */

void PoseslamAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->alg_.unlock();
}

void PoseslamAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<PoseslamAlgNode>(argc, argv, "poseslam_alg_node");
}

void PoseslamAlgNode::augment_trajectories(const std_msgs::Header& header)
{
  if (Trajectory_msg_.poses.size() == Trajectory_msg_.poses.capacity())
    Trajectory_msg_.poses.reserve(Trajectory_msg_.poses.size() + 100);
  
  if (Trajectory_msg_.steps_2_states.size() == Trajectory_msg_.steps_2_states.capacity())
    Trajectory_msg_.steps_2_states.reserve(Trajectory_msg_.steps_2_states.size() + 100);
  
  if (Trajectory_poses_.size() == Trajectory_poses_.capacity())
    Trajectory_poses_.reserve(Trajectory_poses_.size() + 100);

  // HEADER
  Trajectory_msg_.header.seq = header.seq;
  Trajectory_msg_.header.stamp = header.stamp;//ros::Time::now();
  Trajectory_msg_.header.frame_id = "/map";
   
  // STEPS AND STATES suposed redundant
  Trajectory_msg_.steps_2_states.push_back(-1);
  
  // LAST POSE
  Trajectory_msg_.poses.push_back(create_PoseWithCovarianceStamped(header, alg_.get_last_pose(), alg_.get_last_covariance()));
  Trajectory_poses_.push_back(alg_.get_last_pose());
}

void PoseslamAlgNode::update_trajectories(const bool& LoopClosed)
{  
  if (Trajectory_msg_.states_2_steps.size() == Trajectory_msg_.states_2_steps.capacity())
    Trajectory_msg_.states_2_steps.reserve(Trajectory_msg_.states_2_steps.size() + 100);

  // STEPS AND STATES CORRECTION
  if (!alg_.is_redundant())
  {
    Trajectory_msg_.states_2_steps.push_back(step_);
    Trajectory_msg_.steps_2_states.back() = Trajectory_msg_.states_2_steps.size() - 1;
    
    ROS_DEBUG("POSE SLAM: Update traj: STATE %u - step_ %u", uint(Trajectory_msg_.states_2_steps.size() - 1), step_);
  }

  // LOOP CLOSURE
  if (LoopClosed)
    recompute_trajectory();
}

void PoseslamAlgNode::recompute_trajectory()
{
  // Get trajectory data
  std::vector<VectorXd> non_redundant_trajectory_poses = alg_.get_trajectory();
  std::vector<std::vector<double> > non_redundant_trajectory_cov = alg_.get_trajectory_covariance();
  std::vector<double> empty_cov (9,0);
  
  // Update new trajectory poses
  for (uint i = 0; i < Trajectory_msg_.states_2_steps.size() - 1; i++)
  {
    uint initial_step = Trajectory_msg_.states_2_steps.at(i);
    uint last_step = Trajectory_msg_.states_2_steps.at(i + 1);
    
    // Recompute segment between non redundant poses
    std::vector<VectorXd> new_segment = recompute_segment(non_redundant_trajectory_poses.at(i), non_redundant_trajectory_poses.at(i + 1), initial_step, last_step);
    //ROS_INFO("POSE SLAM: Recomputed segment between %i and %i: %f, %f, %f and %f, %f, %f", i, i+1, non_redundant_trajectory_poses.at(i)(0),non_redundant_trajectory_poses.at(i)(1),non_redundant_trajectory_poses.at(i)(2), non_redundant_trajectory_poses.at(i + 1)(0), non_redundant_trajectory_poses.at(i + 1)(1), non_redundant_trajectory_poses.at(i + 1)(2));

    // Update the state i
    Trajectory_msg_.poses.at(initial_step).pose = create_PoseWithCovariance(non_redundant_trajectory_poses.at(i), non_redundant_trajectory_cov.at(i));
    //ROS_INFO("POSE SLAM: recomputed non-redundant step_: %i", initial_step);
    
    // Update the segment intermediate poses
    for (uint j = 0; j < new_segment.size(); j++)
    {
      //ROS_INFO("POSE SLAM: recomputed redundant step_: %i", initial_step + j + 1);
      Trajectory_msg_.poses.at(initial_step + j + 1).pose = create_PoseWithCovariance(new_segment.at(j), empty_cov);
      Trajectory_poses_.at(initial_step + j + 1) = new_segment.at(j);
    }
  }

  // Update last state
  Trajectory_msg_.poses.back().pose = create_PoseWithCovariance(non_redundant_trajectory_poses.back(), non_redundant_trajectory_cov.back());
  //ROS_INFO("POSE SLAM: Recomputed step_ %i: %f, %f, %f", Trajectory_msg_.poses.size() - 1, non_redundant_trajectory_poses.back()(0), non_redundant_trajectory_poses.back()(1),non_redundant_trajectory_poses.back()(2));
  //ROS_INFO("POSE SLAM: recomputed non-redundant step_: %i", Trajectory_msg_.poses.size() - 1);
    
  // Update loops indexs
  if (Trajectory_msg_.loops.size() == Trajectory_msg_.loops.capacity())
    Trajectory_msg_.loops.reserve(Trajectory_msg_.loops.size() + 100);
  
  for (uint i = Trajectory_msg_.loops.size(); i < loops_.size(); i++)
    Trajectory_msg_.loops.push_back(loops_.at(i));
  
  // Store the trajectory non-redundant poses
  for (uint i = 0; i < Trajectory_msg_.states_2_steps.size(); i++)
    Trajectory_poses_.at(Trajectory_msg_.states_2_steps.at(i)) = non_redundant_trajectory_poses.at(i);
    
  }

std::vector<VectorXd> PoseslamAlgNode::recompute_segment(const VectorXd& new_initial_pose, const VectorXd& new_final_pose, const int& initial_step, const int& final_step)
{
  std::vector<VectorXd> segment_poses(final_step - initial_step - 1);
  // Old segment poses
  std::vector<VectorXd> old_segment_poses;
  old_segment_poses.assign(Trajectory_poses_.begin() + initial_step + 1, Trajectory_poses_.begin() + final_step);
  // Old segment displacement
  VectorXd old_d = Trajectory_poses_.at(final_step) - Trajectory_poses_.at(initial_step);
  old_d(2) = pi_2_pi(old_d(2));
  // New segment displacement
  VectorXd new_d = new_final_pose - new_initial_pose;
  new_d(2) = pi_2_pi(new_d(2));
  
  // Transformation
  double xy_scale = (old_d.head(2).norm() == 0 ? 1: new_d.head(2).norm() / old_d.head(2).norm());
  double th_scale = (old_d(2) == 0 ? 1: new_d(2) / old_d(2));
  double rotation = atan2(old_d(0), old_d(1)) - atan2(new_d(0), new_d(1));
  MatrixXd R = rotation_matrix(rotation);

  for (uint i = 0; i < old_segment_poses.size(); i++)
  {
    // vector from initial pose
    VectorXd d_i = old_segment_poses.at(i) - Trajectory_poses_.at(initial_step);
    // scale
    d_i.head(2) = xy_scale * d_i.head(2);
    d_i(2) = pi_2_pi(th_scale * d_i(2));
    // xy rotation
    d_i = R * d_i;
    // intermediate pose
    segment_poses.at(i) = new_initial_pose + d_i;
    //ROS_INFO("PS: intermediate pose: %f, %f,%f", segment_poses.at(i)(0), segment_poses.at(i)(1), segment_poses.at(i)(2));

    assert( ((segment_poses.at(i) - segment_poses.at(i)).array() == (segment_poses.at(i) - segment_poses.at(i)).array()).all() ); // contains finite numbers
  }
  return segment_poses;
}

geometry_msgs::PoseWithCovarianceStamped PoseslamAlgNode::create_PoseWithCovarianceStamped(const std_msgs::Header& header, const VectorXd& last_pose, const std::vector<double>& last_cov)
{
  geometry_msgs::PoseWithCovarianceStamped new_pose;
  new_pose.header = header;
  new_pose.header.frame_id = "/map";
  new_pose.pose = create_PoseWithCovariance(last_pose, last_cov);
  return new_pose;
}

geometry_msgs::PoseWithCovariance PoseslamAlgNode::create_PoseWithCovariance(const VectorXd& last_pose, const std::vector<double>& last_cov)
{
  geometry_msgs::PoseWithCovariance new_pose;
  new_pose.pose.position.x = last_pose(0);
  new_pose.pose.position.y = last_pose(1);
  new_pose.pose.position.z = 0;
  new_pose.pose.orientation = tf::createQuaternionMsgFromYaw(last_pose(2));
  for (uint j = 0; j < 9; j++)
    new_pose.covariance.at(j) = last_cov.at(j); 
  return new_pose;
}

MatrixXd PoseslamAlgNode::rotation_matrix(const double &alpha) const
{
  MatrixXd rot = MatrixXd::Identity(3,3);
  rot(0,0) = cos(alpha);
  rot(0,1) = -sin(alpha);
  rot(1,0) = sin(alpha);
  rot(1,1) = cos(alpha);
  return rot;
}

double PoseslamAlgNode::pi_2_pi(const double& angle) const
{
  return angle - 2 * M_PI * floor((angle + M_PI)/(2 * M_PI));
}
