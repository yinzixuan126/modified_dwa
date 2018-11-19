#include "akp_local_planner_car_alg_node.h"
#include <pluginlib/class_list_macros.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_DECLARE_CLASS(iri_akp_local_planner, 
                        AkpLocalPlanner, 
                        AkpLocalPlanner, 
                        nav_core::BaseLocalPlanner)

AkpLocalPlanner::AkpLocalPlanner(void) :
  fscan_received(false), 
  rscan_received(false), 
  planner_(5.0,500, Cplan_local_nav::F_RRT_GC_alpha), 
  costmap_ros_(NULL), 
  tf_(NULL), 
  initialized_(false), 
  setup_(false), 
  vis_mode_(1),
  frozen_mode_(false), 
  move_base(true),
  text_markers_old_size_(0),
  slicing_path_diff_orientation_(20.0)
{
  pthread_mutex_init(&this->planner_mutex_,NULL);
  pthread_mutex_init(&this->fscan_mutex_,NULL);
  pthread_mutex_init(&this->rscan_mutex_,NULL);
  pthread_mutex_init(&this->tracks_mutex_,NULL);
  pthread_mutex_init(&this->odom_mutex_,NULL);
  pthread_mutex_init(&this->params_values_mutex_,NULL);
  
  //init class attributes if necessary
  
  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
  init();
}

void AkpLocalPlanner::planner_mutex_enter(void){
  pthread_mutex_lock(&this->planner_mutex_);
}
void AkpLocalPlanner::planner_mutex_exit(void){
  pthread_mutex_unlock(&this->planner_mutex_);
}
void AkpLocalPlanner::fscan_mutex_enter(void){
  pthread_mutex_lock(&this->fscan_mutex_);
}
void AkpLocalPlanner::fscan_mutex_exit(void){
  pthread_mutex_unlock(&this->fscan_mutex_);
}
void AkpLocalPlanner::rscan_mutex_enter(void){
  pthread_mutex_lock(&this->rscan_mutex_);
}
void AkpLocalPlanner::rscan_mutex_exit(void){
  pthread_mutex_unlock(&this->rscan_mutex_);
}
void AkpLocalPlanner::odom_mutex_enter(void){
  pthread_mutex_lock(&this->odom_mutex_);
}
void AkpLocalPlanner::odom_mutex_exit(void){
  pthread_mutex_unlock(&this->odom_mutex_);
}
void AkpLocalPlanner::tracks_mutex_enter(void){
  pthread_mutex_lock(&this->tracks_mutex_);
}
void AkpLocalPlanner::tracks_mutex_exit(void){
  pthread_mutex_unlock(&this->tracks_mutex_);
}
void AkpLocalPlanner::params_values_mutex_enter(void){
  pthread_mutex_lock(&this->params_values_mutex_);
}
void AkpLocalPlanner::params_values_mutex_exit(void){
  pthread_mutex_unlock(&this->params_values_mutex_);
}


AkpLocalPlanner::~AkpLocalPlanner(void)
{
  // [free dynamic memory]
  //make sure to clean things up
  delete dsrv_;
  pthread_mutex_destroy(&this->planner_mutex_);
  pthread_mutex_destroy(&this->fscan_mutex_);
  pthread_mutex_destroy(&this->rscan_mutex_);
  pthread_mutex_destroy(&this->tracks_mutex_);
  pthread_mutex_destroy(&this->odom_mutex_);
  pthread_mutex_destroy(&this->params_values_mutex_);
}

void AkpLocalPlanner::init()
{


    this->cmd_vel_publisher_ = this->public_node_handle_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    this->markers_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("planner_markers", 100);
    this->cost_params_publisher_ = this->public_node_handle_.advertise<std_msgs::Float64MultiArray>("cost_values", 10);

    this->fscan_subscriber_   = this->public_node_handle_.subscribe<sensor_msgs::LaserScan>("front_scan" , 1, boost::bind(&AkpLocalPlanner::fscan_callback, this, _1));
    this->rscan_subscriber_   = this->public_node_handle_.subscribe<sensor_msgs::LaserScan>("rear_scan" , 1,  boost::bind(&AkpLocalPlanner::rscan_callback, this, _1));
    this->odom_subscriber_   = this->public_node_handle_.subscribe<nav_msgs::Odometry>("odom"  , 1, boost::bind(&AkpLocalPlanner::odom_callback, this, _1));
    this->tracks_subscriber_ = this->public_node_handle_.subscribe<iri_perception_msgs::detectionArray>("tracks", 1, boost::bind(&AkpLocalPlanner::tracks_callback, this, _1));
    this->params_values_subscriber_ = this->public_node_handle_.subscribe<std_msgs::Float64MultiArray>("params", 1, boost::bind(&AkpLocalPlanner::params_values_callback, this, _1));


       
    Float64_msg_.data.resize(8,0.0);//TODO cuantos elementos?


}
void AkpLocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if(!initialized_)
  {
    this->tf_          = tf;
    this->costmap_ros_ = costmap_ros;
    ros::NodeHandle private_nh("~/" + name);
    //private publishers
    g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
    l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
    
        //Initialize force planner parameters
    private_nh.getParam("xy_goal_tolerance", this->xy_goal_tolerance);
    private_nh.getParam("robot", this->robot_);
    double horizon_time, v_max, w_max, av_max, av_break, aw_max, platform_radii;
    double cost_distance,cost_orientation,cost_w_robot,cost_w_people,cost_time, cost_obs, cost_old_path, cost_l_minima;
    int mode, nvertex;
    this->planner_.set_dt(0.2);//TODO depends upon the move_base frame_rate
    
    //robot frames
    this->fixed_frame = "/map";
    this->robot_frame  = "/" + this->robot_ + "/base_link";

    private_nh.getParam("move_base", this->move_base);
    private_nh.getParam("plan_mode", mode);
    this->plan_mode_ = (Cplan_local_nav::plan_mode)mode;
    this->planner_.set_planning_mode(this->plan_mode_);
    private_nh.getParam("distance_mode", mode);
    this->planner_.set_distance_mode((Cplan_local_nav::distance_mode)mode );
    private_nh.getParam("global_mode", mode);
    this->planner_.set_global_mode((Cplan_local_nav::global_mode)mode );
    
    private_nh.getParam("vis_mode", mode);
    this->vis_mode_ = mode;
    private_nh.getParam("number_vertex", nvertex);
    this->planner_.set_number_of_vertex(nvertex);

    private_nh.getParam("horizon_time", horizon_time);
    this->planner_.set_horizon_time(horizon_time);

    private_nh.getParam("cost_distance",  cost_distance );
    private_nh.getParam("cost_orientation", cost_orientation);
    private_nh.getParam("cost_w_robot",  cost_w_robot);
    private_nh.getParam("cost_w_people",  cost_w_people);
    private_nh.getParam("cost_time",  cost_time);
    private_nh.getParam("cost_obs",  cost_obs);
    private_nh.getParam("cost_old_path",  cost_old_path);
    private_nh.getParam("cost_l_minima",  cost_l_minima);
    this->planner_.set_plan_cost_parameters(cost_distance,
                              cost_orientation,
                              cost_w_robot,
                              cost_w_people,
                              cost_time,
                              cost_obs,
                              cost_old_path,
                              cost_l_minima);

    private_nh.getParam("v_max",  v_max);
    private_nh.getParam("w_max",  w_max);
    private_nh.getParam("av_max", av_max);
    private_nh.getParam("av_min", av_break);
    private_nh.getParam("aw_max", aw_max);
    private_nh.getParam("platform_radii", platform_radii);
    this->planner_.set_robot_params(v_max, w_max, av_max, av_break, aw_max, platform_radii);

    //config ESFM paramters
    std::vector<double> params(5,0.0);
    private_nh.getParam("esfm_k", params[0]);
    private_nh.getParam("esfm_to_person_lambda",params[1] );
    private_nh.getParam("esfm_to_person_A",params[2] );
    private_nh.getParam("esfm_to_person_B", params[3]);
    private_nh.getParam("esfm_d", params[4]);
    this->planner_.set_sfm_to_person( params );
    private_nh.getParam("esfm_to_robot_lambda",params[1] );
    private_nh.getParam("esfm_to_robot_A", params[2]);
    private_nh.getParam("esfm_to_robot_B", params[3]);
    planner_.set_sfm_to_robot( params );
    private_nh.getParam("esfm_to_obstacle_lambda",params[1] );
    private_nh.getParam("esfm_to_obstacle_A", params[2]);
    private_nh.getParam("esfm_to_obstacle_B", params[3]);
    this->planner_.set_sfm_to_obstacle( params );
    
    double min_v_to_predict;
    int ppl_col, pr_force_mode;
    private_nh.getParam("min_v_to_predict", min_v_to_predict);
    this->planner_.set_min_v_to_predict( min_v_to_predict );//Cscene_bhmip
    private_nh.getParam("ppl_collision_mode", ppl_col);
    this->planner_.set_ppl_collision_mode( ppl_col );
    private_nh.getParam("pr_force_mode", pr_force_mode);
    this->planner_.set_pr_force_mode( pr_force_mode );
    int mod;
    private_nh.getParam("goal_providing_mode", mod);
    goal_providing_mode_ = (AkpLocalPlanner::Goal_providing_mode)mod;
    private_nh.getParam("slicing_diff_orientation", slicing_path_diff_orientation_);

    private_nh.getParam("force_map_path", this->force_map_path_);
    private_nh.getParam("destination_map_path", this->destination_map_path_);

    ROS_INFO("AkpLocalPlanner::initialize: Destinations map %s", destination_map_path_.c_str());
    if(!planner_.read_destination_map(destination_map_path_.c_str()))
      ROS_ERROR("AkpLocalPlanner::initialize: Could not read map destinations file!!!");
    else
      ROS_INFO("AkpLocalPlanner::initialize: Read destinations map file SUCCESS!!!");

    ROS_INFO("AkpLocalPlanner::initialize: Force map %s", force_map_path_.c_str());
    if(!planner_.read_force_map(force_map_path_.c_str()))
      ROS_ERROR("AkpLocalPlanner::initialize: Could not read map force file!!!");
    else
      ROS_INFO("AkpLocalPlanner::initialize: Read map force file SUCCESS!!!");
    

    init_force_planner_and_markers();

    dsrv_ = new dynamic_reconfigure::Server<iri_akp_local_planner::AkpLocalPlannerConfig>(private_nh);
    dynamic_reconfigure::Server<iri_akp_local_planner::AkpLocalPlannerConfig>::CallbackType cb = boost::bind(&AkpLocalPlanner::reconfigureCallback, this, _1, _2);
    dsrv_->setCallback(cb);
    initialized_ = true;
  }
  else
    ROS_WARN("AkpLocalPlanner::initialize: This planner has already been initialized, you can't call it twice, doing nothing");
}

bool AkpLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  //ROS_INFO("AkpLocalPlanner::computeVelocityCommands");
  if(!initialized_)
  {
    ROS_ERROR("AkpLocalPlanner::computeVelocityCommands: This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  this->planner_mutex_enter();

  //update front and rear obstacles
  this->laser_points.clear();

  this->fscan_mutex_enter();
  if(this->fscan_received)
  {
    this->laser_points = this->scan2points(this->fscan);
    this->fscan_received=false;
  }
  this->fscan_mutex_exit();

  this->rscan_mutex_enter();
  if(this->rscan_received)
  {
    std::vector< Spoint > rear_points;
    rear_points = this->scan2points(this->rscan);
    this->laser_points.insert(this->laser_points.end(), rear_points.begin(), rear_points.end());
    this->rscan_received=false;
  }
  this->rscan_mutex_exit();

  this->planner_.read_laser_scan( this->laser_points );

  //update robot position
  this->odom_mutex_enter();
  this->planner_.update_robot(this->robot_pose_);
  this->odom_mutex_exit();
  
  
  //update people observations
  this->tracks_mutex_enter();
  this->planner_.update_scene(this->obs);
  this->obs.clear();//when no updates are done, this message should be clear, and not the previous
  this->tracks_mutex_exit();

  //planner iteration
  double t = ros::Time::now().toSec();
  Spose best_next_pose;
  bool robot_plan_succed =  this->planner_.robot_plan(best_next_pose);
  ROS_INFO( "robot plan  %s in time %f", robot_plan_succed ? "true" : "false", ros::Time::now().toSec()-t );
  //ROS_INFO( "robot plan  %s at velocity %f", robot_plan_succed ? "true" : "false",  best_next_pose.v );
 

  //Heuristic to average velocities and detect local minima problems
  if(this->move_base)
    velocities_.push_back( robot_pose_.v );
  //pop front old velocities; in setPlan, a reset of the vector is done: 5 samples ~ 1seg
  if( velocities_.size() > 5 )
    velocities_.pop_front();
    
  // calculate average
  double avg_v(0.0);
  for( unsigned int i = 0; i < velocities_.size(); ++i )
    avg_v += velocities_[i];
  //detects abnormal behavior, that is, stopping robot
  //ROS_INFO("avergae velocity = %f of size %d", avg_v/5.0, (int)velocities_.size());
  if( fabs(avg_v) < 5.0*0.1 && velocities_.size() > 4 )
  {
    robot_plan_succed = false;
    ROS_INFO( "robot plan invalidated");
  }


  if(this->move_base)
  {
    cmd_vel.linear.x  = best_next_pose.v;
    cmd_vel.angular.z = best_next_pose.w;
  }
  else
  {
    cmd_vel.linear.x  = 0.0;
    cmd_vel.angular.z = 0.0;
  }

  publishPlan(this->global_plan_, g_plan_pub_);
  publishPlan(this->local_plan_, l_plan_pub_);
  //t = ros::Time::now().toSec();//just to measure the drawing of markers
  //fill planning markers
  if( vis_mode_ > 0 && !frozen_mode_  )
  {
    MarkerArray_msg_.markers.clear();
    //minimal mode: scene and best path
      //ROS_INFO( "entering fill best path 2d" );
    fill_best_path_2d();
      //ROS_INFO( "entering scene markers" );
    fill_scene_markers();
    if( vis_mode_ > 1 )//normal mode 2d features
    {
      fill_forces_markers();
      fill_laser_obstacles();
      fill_planning_markers_2d();
    }
    if ( vis_mode_ > 2 )//super mode 3d features
    {
      fill_people_prediction_markers_2d();
      fill_people_prediction_markers_3d();
      fill_planning_markers_3d();
    }
    if ( vis_mode_ > 3 )//super mode, plooting potentials and so on
    {
      //TODO
    }
  }
  this->markers_publisher_.publish(this->MarkerArray_msg_);
  
  // publish robot costs TODO to be deprecated
  //double work_robot, work_persons;
  //this->planner_.get_navigation_instant_work( work_robot, work_persons );
  //Float64_msg_.data[0] = work_robot;
  //ROS_DEBUG("FoceLocalPlanner::computeVelocityCommands: cmd_vel=(%f,%f)", cmd_vel.linear.x,  cmd_vel.angular.z);
  
  // publish navigation costs dist[0], orient[1], robot[2], ppl[3], obst[4]
    //ROS_INFO( "entering costs" );
  std::vector<double> costs;
  this->planner_.get_navigation_cost_values( costs);
  Float64_msg_.data = costs;
  this->planner_.get_navigation_mean_cost_values( costs);
  Float64_msg_.data.insert( Float64_msg_.data.end(), costs.begin(), costs.end() );
  this->planner_.get_navigation_std_cost_values( costs);
  Float64_msg_.data.insert( Float64_msg_.data.end(), costs.begin(), costs.end() );
  this->cost_params_publisher_.publish(this->Float64_msg_);
  //ROS_INFO( "current markers = %f", ros::Time::now().toSec()-t);
  this->planner_mutex_exit();
  //ROS_INFO("AkpLocalPlanner::ComputeVelocites: Exit"); 
  return robot_plan_succed;
}

bool AkpLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  //ROS_INFO("AkpLocalPlanner::setPlan");
  if(!initialized_)
  {
    ROS_ERROR("AkpLocalPlanner::setPlan: This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }
  velocities_.clear();//clears the average of velocities when new plans are calculated
  this->global_plan_.clear();
  this->global_plan_ = orig_global_plan;
  ROS_INFO("AkpLocalPlanner::setPlan: global_plan_ points %d", uint(this->global_plan_.size()));
  // slice global plan into subgoals
  if ( goal_providing_mode_ == AkpLocalPlanner::Slicing_global )
    slice_plan( );
  //else , there is no need to do anything
  return true;
}

void AkpLocalPlanner::slice_plan()
{
  sliced_global_plan_.clear();
  std::vector<double> orientations, avg_orientations;
  unsigned int n(5), i_down, i_up;
  double dx, dy, theta, accumulated_orientation, avg_ori;
  
  // get vector of orientaions in 2D-> yaw angle
  assert(orientations.size() <=1 && "AkpLocalPlanner::slice_plan: global_plan vector empty");
  for( unsigned int i=1; i < global_plan_.size(); ++i )
  {
    /*fake orientation message, filled with constant q=[0,0,0,1] o.O
    ROS_INFO( "quaternion entering = (%f, %f, %f, %f)" ,global_plan_[i].pose.orientation.x,
                      global_plan_[i].pose.orientation.y,
                      global_plan_[i].pose.orientation.z,
                      global_plan_[i].pose.orientation.w );*/
    
    //differential in 2d
    dy = global_plan_[i].pose.position.y - global_plan_[i-1].pose.position.y;
    dx = global_plan_[i].pose.position.x - global_plan_[i-1].pose.position.x;
    orientations.push_back( atan2( dy, dx ) );
    //ROS_INFO("2d processing = %f", theta);
  }
  
  
  assert(!orientations.empty() && "AkpLocalPlanner::slice_plan: orientations vector empty");
  //calculate the differential of orientations in 2D in [-pi, pi]
  for(unsigned int i=1; i < orientations.size(); ++i)
  {
    orientations[i-1] = orientations[i] - orientations[i-1];
    if( orientations[i-1] > PI )
      orientations[i-1] -= 2.0*PI;
    if (orientations[i-1] < -PI )
      orientations[i-1] += 2.0*PI;
  }
  orientations.back() = 0.0;
  
  //smooth variantions of orientations and absolute value [0,pi]
  /*
  for(unsigned int i=0; i < orientations.size(); ++i)
  {
    if( i > n ) i_down = i-n;
    else i_down = 0;
    if ( i + n < orientations.size() ) i_up = i+n;
    else i_up = orientations.size();
    avg_ori = 0.0;
    for ( unsigned int j = i_down; j < i_up; ++j )
    {
      avg_ori += orientations[j];
    }
    avg_orientations.push_back( fabs(avg_ori/(double) n) );
    
  } not necessary now*/
  
  //slice the std::vector<geometry_msgs::PoseStamped> global_plan_
  //ROS_INFO("accumulated orientation = %d", orientations.size());
  //initial pose is the last of the plan TODO esto va mal, peta y se pasa del i
  sliced_global_plan_.push_back( global_plan_.back() );
  accumulated_orientation = 0.0;
  for(unsigned int i=orientations.size()-1; i > 0 ; --i)
  {
    accumulated_orientation += fabs(orientations[i]);
    //ROS_INFO("accumulated orientation = %f with respect to %f in %d" , accumulated_orientation, 20.0*PI/180.0, i);
    if ( accumulated_orientation > slicing_path_diff_orientation_*PI/180.0 )
    {
      //check if it near the previous on and then discard
      dx = sliced_global_plan_.back().pose.position.x - global_plan_[i].pose.position.x;
      dy = sliced_global_plan_.back().pose.position.y - global_plan_[i].pose.position.y;
      accumulated_orientation = 0.0;
      if ( dx*dx + dy*dy > 1.0 )
      {
        sliced_global_plan_.push_back( global_plan_[i] );
        //ROS_INFO( "Subgoal in (%f, %f) at i = %d", sliced_global_plan_.back().pose.position.x, sliced_global_plan_.back().pose.position.y, i );
      }
    }  
  }
  
  //for( unsigned int i=0; i < sliced_global_plan_.size(); ++i )
  //  ROS_INFO( "AkpLocalPlanner::slice_plan: Subgoal %d in (%f, %f)", i, sliced_global_plan_[i].pose.position.x, sliced_global_plan_[i].pose.position.y );
      
}

bool AkpLocalPlanner::isGoalReached()
{
  //ROS_INFO("AkpLocalPlanner::isGoalReached");
  bool ok=false;
  double goal_x=0.0;
  double goal_y=0.0;

  if(!initialized_){
    ROS_ERROR("AkpLocalPlanner::isGoalReached: This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  //TODO switch to different subgoal generation: 1) croping at local window. 2) calculating the set of subgoals
  if( goal_providing_mode_ == AkpLocalPlanner::Slicing_global )
  {
    //propose the new goal as the last of
    robot_goal_.x = sliced_global_plan_.back().pose.position.x;
    robot_goal_.y = sliced_global_plan_.back().pose.position.y;  
    //identify of it is the last subgoal is reached and then propose a new one if any
    if( this->robot_pose_.distance( this->robot_goal_ ) < planner_.get_distance_to_stop() &&
        sliced_global_plan_.size() > 1  )
    {
      sliced_global_plan_.pop_back();
      robot_goal_.x = sliced_global_plan_.back().pose.position.x;
      robot_goal_.y = sliced_global_plan_.back().pose.position.y;  
    }
  }
  else // if( goal_providing_mode_ == AkpLocalPlanner::Crop_local_window)
  {
  
    //transform plan from global to local coordinates, and crop it to the local costmap window
    std::string base_frame_ = this->robot_frame;
    if(!transformGlobalPlan(*this->tf_, this->global_plan_, *this->costmap_ros_, base_frame_, this->local_plan_))
    {
      ROS_WARN("AkpLocalPlanner::isGoalReached(): Could not transform the global plan to the frame of the controller, %s", base_frame_.c_str());
      return false;
    }

    //transform last pose of the local plan to global coordinates
    geometry_msgs::PoseStamped last_pose(this->local_plan_.back());
    std::string global_frame_ = this->fixed_frame;
    if(!transformPose(*this->tf_, last_pose, *this->costmap_ros_, global_frame_, last_pose))
    {
      ROS_WARN("AkpLocalPlanner::isGoalReached(): Could not transform the goal to the frame of the controller, %s", base_frame_.c_str());
      return false;
    }

    //send that pose as goal
    tf::Stamped<tf::Pose> goal_point;
    tf::poseStampedMsgToTF(last_pose, goal_point);
    goal_x  = goal_point.getOrigin().getX();
    goal_y  = goal_point.getOrigin().getY();
    //double yaw    = tf::getYaw(goal_point.getRotation());
    this->robot_goal_.x = goal_x;
    this->robot_goal_.y = goal_y;
  }//end of else, case for the Crop_local_window

  this->planner_mutex_enter();
  this->planner_.set_robot_goal(this->robot_goal_);

  if(this->robot_pose_.distance( this->robot_goal_ ) < this->xy_goal_tolerance
     && this->robot_pose_.v < this->v_goal_tolerance)
  {
    ok=true;
    ROS_INFO("AkpLocalPlanner::isGoalReached: GOAL REACHED x,y: %f, %f at distance %f", robot_goal_.x, robot_goal_.y, this->robot_pose_.distance( this->robot_goal_ ));
  }

  this->planner_mutex_exit();

  //ROS_DEBUG("AkpLocalPlanner::isGoalReached: x,y: %f, %f and size = %d", robot_goal_.x, robot_goal_.y, (int)sliced_global_plan_.size());
  return ok;
}

/*  [subscriber callbacks] */
void AkpLocalPlanner::fscan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
  //ROS_INFO("AkpLocalPlanner::scan_callback: New Message Received"); 

  this->fscan_mutex_enter(); 
  this->fscan = *msg;
  this->fscan_received=true;
  this->fscan_mutex_exit(); 
}

void AkpLocalPlanner::rscan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
  //ROS_INFO("AkpLocalPlanner::scan_callback: New Message Received"); 

  this->rscan_mutex_enter(); 
  this->rscan = *msg;
  this->rscan_received=true;
  this->rscan_mutex_exit(); 
}

std::vector<Spoint> AkpLocalPlanner::scan2points(const sensor_msgs::LaserScan scan)
{

  std::vector<Spoint> points;
  std::string source_frame = scan.header.frame_id;
  std::string target_frame = this->fixed_frame;
  ros::Time target_time    = scan.header.stamp;
  try
  {
    bool tf_exists = this->tf_->waitForTransform(target_frame, source_frame, target_time, ros::Duration(1), ros::Duration(0.01));
    if(tf_exists)
    {
      geometry_msgs::PointStamped pointIn;
      geometry_msgs::PointStamped pointOut;
      pointIn.header = scan.header;
      double t;
      for( unsigned int i = 0; i < scan.ranges.size(); ++i)
      {
        if ( scan.ranges[i]  < 15.0  && scan.ranges[i] > 0.05)
        {
          t = scan.angle_min + (double)i*scan.angle_increment;
          pointIn.point.x = scan.ranges[i] * cos( t );
          pointIn.point.y = scan.ranges[i] * sin( t );
          this->tf_->transformPoint(target_frame, pointIn, pointOut);
          points.push_back( Spoint( pointOut.point.x, pointOut.point.y ) );
        }
      }
    }
    else
    {
      ROS_WARN("AkpLocalPlanner::scan2points: No transform: %s-->%s", source_frame.c_str(), target_frame.c_str());
    }
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("AkpLocalPlanner::scan2points: %s",ex.what());
  }
  return points;
}

void AkpLocalPlanner::odom_callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
  //ROS_INFO("AkpLocalPlanner::odom_callback: New Message Received");

  this->odom_mutex_enter();
  this->robot_pose_.v = msg->twist.twist.linear.x;
  this->robot_pose_.w = msg->twist.twist.angular.z;

  //Get Robot position: transform empty/zero pose from base_link to map
  std::string target_frame = this->fixed_frame;
  std::string source_frame = this->robot_frame; 
  ros::Time target_time    = ros::Time::now();
  try
  {
    bool tf_exists = this->tf_->waitForTransform(target_frame, source_frame, target_time, ros::Duration(1), ros::Duration(0.01));
    if(tf_exists)
    {
      geometry_msgs::PoseStamped poseIn;
      geometry_msgs::PoseStamped poseOut;
      poseIn.header.stamp    = target_time;
      poseIn.header.frame_id = source_frame;
      poseIn.pose.orientation.z = 1.0; //valid quaternion
      this->tf_->transformPose(target_frame, poseIn, poseOut);
      this->robot_pose_.x = poseOut.pose.position.x;
      this->robot_pose_.y = poseOut.pose.position.y;
      this->robot_pose_.time_stamp = poseOut.header.stamp.toSec();

      //vector of the orientation
      poseIn.pose.position.x = 1.0;
      this->tf_->transformPose(target_frame, poseIn, poseOut);
      this->robot_pose_.theta = atan2(poseOut.pose.position.y - this->robot_pose_.y , poseOut.pose.position.x - this->robot_pose_.x);
      //this->planner_.update_robot(this->robot_pose_);//updated in main
      //ROS_DEBUG("AkpLocalPlanner::odom_callback: robot pose x,y,th,v,w: %f, %f, %f, %f, %f", this->robot_pose_.x, this->robot_pose_.y, this->robot_pose_.theta, this->robot_pose_.v, this->robot_pose_.w);
    }
    else
    {
      ROS_WARN("AkpLocalPlanner::odom_callback: No transform: %s-->%s", source_frame.c_str(), target_frame.c_str());
    }
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("AkpLocalPlanner::odom_callback: %s",ex.what());
  }

  this->odom_mutex_exit();
  //ROS_INFO("AkpLocalPlanner::odom_callback: Exit");
}

void AkpLocalPlanner::tracks_callback(const iri_perception_msgs::detectionArray::ConstPtr& msg) 
{
  //ROS_INFO("AkpLocalPlanner::tracks_callback: New Message Received"); 
  
  if(msg->header.frame_id == this->fixed_frame)
  {
    this->tracks_mutex_enter();
    //std::vector<SdetectionObservation> obs;
    this->obs.clear();
    std::vector<double> cov;
    cov.resize(16,0.0);
    for( unsigned int i = 0; i< msg->detection.size(); ++i)
    {
      //ROS_INFO("AkpLocalPlanner::tracks_callback: id = %d, (x,y) = (%f, %f), (vx,vy) = (%f, %f)", msg->detection[i].id, msg->detection[i].position.x, msg->detection[i].position.y, msg->detection[i].velocity.x, msg->detection[i].velocity.y);
      //covariances conversion from 4x4 to 6x6
      cov[0] = msg->detection[i].covariances[0];//x x
      cov[1] = msg->detection[i].covariances[1];//x y
      cov[2] = msg->detection[i].covariances[3];//x vx
      cov[3] = msg->detection[i].covariances[4];//x vy
      cov[4] = msg->detection[i].covariances[6];//x y
      cov[5] = msg->detection[i].covariances[7];//y y
      cov[6] = msg->detection[i].covariances[9];//y vx
      cov[7] = msg->detection[i].covariances[10];//y vy
      cov[8] = msg->detection[i].covariances[18];//vx x
      cov[9] = msg->detection[i].covariances[19];//vx y
      cov[10] = msg->detection[i].covariances[21];//vx vx
      cov[11] = msg->detection[i].covariances[22];//vx vy
      cov[12] = msg->detection[i].covariances[24];//vy x
      cov[13] = msg->detection[i].covariances[25];//vy y
      cov[14] = msg->detection[i].covariances[27];//vy vx
      cov[15] = msg->detection[i].covariances[28];//vy vy
      this->obs.push_back(SdetectionObservation(msg->detection[i].id, msg->header.stamp.toSec(),
                                          msg->detection[i].position.x,  msg->detection[i].position.y ,
                                          msg->detection[i].velocity.x, msg->detection[i].velocity.y, cov));
    }
    //this->planner_.update_scene(this->obs);//filter = false
    this->tracks_mutex_exit();
    //ROS_INFO("AkpLocalPlanner::tracks_callback: Exit"); 
  }
  else
  {
    ROS_ERROR("AkpLocalPlanner::tracks_callback: tracks are in %s frame instead of %s", msg->header.frame_id.c_str(), this->fixed_frame.c_str());
  }
  
}

void AkpLocalPlanner::params_values_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  //ROS_INFO("AkpLocalPlanner::params_values_callback: new message");
  this->planner_.set_plan_cost_parameters(msg->data[0],//distance
                              msg->data[1],//orientation
                              msg->data[2],//robot
                              msg->data[3],//people
                              0.0,        //time
                              msg->data[4],//obstacles
                              msg->data[5],//past trajetory
                              0.0);//local minima

}


/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void AkpLocalPlanner::reconfigureCallback(iri_akp_local_planner::AkpLocalPlannerConfig &config, uint32_t level)
{
  std::vector<double> params(5,0.0);
  if(setup_ && config.restore_defaults)
  {
    config = default_config_;
    //Avoid looping
    config.restore_defaults = false;
  }
  if(!setup_)
  {
    default_config_ = config;
    setup_ = true;
  }
  else if(setup_)
  {
    this->planner_mutex_enter();
    this->move_base = config.move_base;
    this->plan_mode_ = (Cplan_local_nav::plan_mode) config.plan_mode;
    this->planner_.set_planning_mode(plan_mode_);
    this->planner_.set_distance_mode((Cplan_local_nav::distance_mode)config.distance_mode );
    this->planner_.set_global_mode((Cplan_local_nav::global_mode)config.global_mode );
    this->planner_.set_number_of_vertex(config.number_vertex);
    ROS_INFO("\n\n\n numer of vertex = %d\n\n\n\n" , config.number_vertex);
    this->planner_.set_horizon_time( config.horizon_time );
    this->xy_goal_tolerance = config.xy_goal_tolerance;
    this->planner_.set_xy_2_goal_tolerance( this->xy_goal_tolerance * this->xy_goal_tolerance);
    this->planner_.set_distance_to_stop( config.distance_to_stop );
    this->v_goal_tolerance = config.v_goal_tolerance;
    this->planner_.set_v_goal_tolerance( this->v_goal_tolerance );
    this->planner_.set_plan_cost_parameters(config.cost_distance,
                              config.cost_orientation,
                              config.cost_w_robot,
                              config.cost_w_people,
                              config.cost_time,
                              config.cost_obs,
                              config.cost_old_path,
                              config.cost_l_minima);
    this->planner_.set_robot_params(config.v_max, config.w_max, config.av_max, config.av_break, config.aw_max, config.platform_radii);
    this->vis_mode_ = config.vis_mode;
    this->frozen_mode_ = config.frozen_mode;
    //config ESFM paramters
    params[0] = config.esfm_k;//k
    params[1] = config.esfm_to_person_lambda;//lambda
    params[2] = config.esfm_to_person_A;//A
    params[3] = config.esfm_to_person_B;//B
    params[4] = config.esfm_d;//d
    planner_.set_sfm_to_person( params );
    params[1] = config.esfm_to_robot_lambda;//lambda
    params[2] = config.esfm_to_robot_A;//A
    params[3] = config.esfm_to_robot_B;//B
    planner_.set_sfm_to_robot( params );
    params[1] = config.esfm_to_obstacle_lambda;//lambda
    params[2] = config.esfm_to_obstacle_A;//A
    params[3] = config.esfm_to_obstacle_B;//B
    planner_.set_sfm_to_obstacle( params );
    this->planner_.set_min_v_to_predict( config.min_v_to_predict );//Cscene_bhmip
    this->planner_.set_ppl_collision_mode( config.ppl_collision_mode );
    this->planner_.set_pr_force_mode( config.pr_force_mode );
    goal_providing_mode_ = (AkpLocalPlanner::Goal_providing_mode)config.goal_providing_mode;
    slicing_path_diff_orientation_ = config.slicing_diff_orientation;
    this->planner_mutex_exit();
  }
}

//copied from goal_functions.cpp in base_local_planner
bool AkpLocalPlanner::transformGlobalPlan(const tf::TransformListener& tf, 
                                            const std::vector<geometry_msgs::PoseStamped>& global_plan, 
                                            const costmap_2d::Costmap2DROS& costmap, 
                                            const std::string& global_frame, 
                                            std::vector<geometry_msgs::PoseStamped>& transformed_plan)
{
  bool ok=true;
  transformed_plan.clear();

  try
  {
    if (!global_plan.size() > 0)
    {
      ROS_ERROR("AkpLocalPlanner::transformGlobalPlan: Recieved plan with zero length");
      return false;
    }
    const geometry_msgs::PoseStamped& plan_pose = global_plan[0];
    tf::StampedTransform transform;
    tf.lookupTransform(global_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id, transform);

    //let's get the pose of the robot in the frame of the plan
    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = this->robot_frame;
    robot_pose.stamp_ = ros::Time();
    tf.transformPose(plan_pose.header.frame_id, robot_pose, robot_pose);

    //we'll keep points on the plan that are within the window that we're looking at
    double dist_threshold = planner_.get_workspace_radii();
    if(dist_threshold ==0)
      dist_threshold=0.5;
    unsigned int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = DBL_MAX;
    //we need to loop to a point on the plan that is within a certain distance of the robot
    while(i < (unsigned int)global_plan.size() && sq_dist > sq_dist_threshold)
    {
      double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
      double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      ++i;
    }
    //make sure not to count the first point that is too far away
    if(i > 0)
      --i;

    tf::Stamped<tf::Pose> tf_pose;
    geometry_msgs::PoseStamped newer_pose;

    //now we'll transform until points are outside of our distance threshold
    while(i < (unsigned int)global_plan.size() && sq_dist < sq_dist_threshold)
    {
      double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
      double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      const geometry_msgs::PoseStamped& pose = global_plan[i];
      poseStampedMsgToTF(pose, tf_pose);
      tf_pose.setData(transform * tf_pose);
      tf_pose.stamp_ = transform.stamp_;
      tf_pose.frame_id_ = global_frame;
      poseStampedTFToMsg(tf_pose, newer_pose);
      transformed_plan.push_back(newer_pose);
      ++i;
    }
  }
  catch(tf::LookupException& ex)
  {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ConnectivityException& ex)
  {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ExtrapolationException& ex)
  {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    if (global_plan.size() > 0)
      ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());
    return false;
  }
  if(transformed_plan.size()==0)
    ok=false;
  return ok;
}

bool AkpLocalPlanner::transformPose(const tf::TransformListener& tf, 
                                      const geometry_msgs::PoseStamped& plan_pose, 
                                      const costmap_2d::Costmap2DROS& costmap, 
                                      const std::string& target_frame, 
                                      geometry_msgs::PoseStamped& transformed_pose)
{
  try
  {
    tf::StampedTransform transform;
    tf.lookupTransform(target_frame, ros::Time(), 
                        plan_pose.header.frame_id, plan_pose.header.stamp, 
                        plan_pose.header.frame_id, transform);
    tf.transformPose(target_frame, plan_pose, transformed_pose);
  }
  catch(tf::LookupException& ex)
  {
    ROS_ERROR("AkpLocalPlanner::transformPose: No Transform available Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ConnectivityException& ex)
  {
    ROS_ERROR("AkpLocalPlanner::transformPose: Connectivity Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ExtrapolationException& ex)
  {
    ROS_ERROR("AkpLocalPlanner::transformPose: Extrapolation Error: %s\n", ex.what());
    return false;
  }
  return true;
}

void AkpLocalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub)
{
  //given an empty path we won't do anything
  if(path.empty())
    return;
  //create a path message
  nav_msgs::Path gui_path;
  gui_path.poses.resize(path.size());
  gui_path.header.frame_id = path[0].header.frame_id;
  gui_path.header.stamp = path[0].header.stamp;
  // Extract the plan in world co-ordinates, we assume the path is all in the same frame
  for(unsigned int i=0; i < path.size(); i++)
  {
    gui_path.poses[i] = path[i];
  }
  pub.publish(gui_path);
}

void AkpLocalPlanner::init_force_planner_and_markers()
{

	//target marker. Predicts the trajectories of people
	pred_traj_marker_.header.frame_id = this->fixed_frame;
	pred_traj_marker_.ns = "pred";
	pred_traj_marker_.type = visualization_msgs::Marker::CYLINDER;
	pred_traj_marker_.action = visualization_msgs::Marker::ADD;
	pred_traj_marker_.lifetime = ros::Duration(1.23f);
	pred_traj_marker_.scale.z = planner_.get_dt();
	pred_traj_marker_.color.a = 0.4;
	pred_traj_marker_.color.r = 0.3;
	pred_traj_marker_.color.g = 1.0;
	pred_traj_marker_.color.b = 0.3;
  pred_traj_marker_.pose.orientation.w = 1.0;
  
  //target marker of the best path, projected in 2d
  pred_traj2d_marker_ = pred_traj_marker_;
	pred_traj2d_marker_.ns = "pred2d";
	pred_traj2d_marker_.scale.z = 0.05;
	pred_traj2d_marker_.pose.position.z = 0.025;

  //cylinder marker, for destinations and goals
	cylinder_marker_.header.frame_id = this->fixed_frame;
	cylinder_marker_.ns = "scene";
	cylinder_marker_.type = visualization_msgs::Marker::CYLINDER;
	cylinder_marker_.action = visualization_msgs::Marker::ADD;
	cylinder_marker_.lifetime = ros::Duration(1.3f);
	cylinder_marker_.color.a = 1.0;
	cylinder_marker_.color.r = 0.3;
	cylinder_marker_.color.g = 0.1;
	cylinder_marker_.color.b = 0.8;
  cylinder_marker_.scale.x = 0.7;
  cylinder_marker_.scale.y = 0.7;
	cylinder_marker_.scale.z = 1.2;
  cylinder_marker_.pose.position.z = 0.6;
  cylinder_marker_.pose.orientation.w = 1.0;

  // robot goal marker, cylinders plot in the scene
	robot_goal_marker_.header.frame_id = this->fixed_frame;
	robot_goal_marker_.ns = "scene";
	robot_goal_marker_.type = visualization_msgs::Marker::CYLINDER;
	robot_goal_marker_.action = visualization_msgs::Marker::ADD;
	robot_goal_marker_.lifetime = ros::Duration(1.3f);
	robot_goal_marker_.color.a = 1.0;
	robot_goal_marker_.color.r = 0.9;
	robot_goal_marker_.color.g = 0.3;
	robot_goal_marker_.color.b = 0.3;
  robot_goal_marker_.scale.x = 0.4;
  robot_goal_marker_.scale.y = 0.4;
	robot_goal_marker_.scale.z = 0.2;
  robot_goal_marker_.pose.position.z = 0.1;
  robot_goal_marker_.pose.orientation.w = 1.0;
  
  robot_subgoal_marker_ = robot_goal_marker_;
	robot_subgoal_marker_.color.r = 0.9;
	robot_subgoal_marker_.color.g = 0.6;
	robot_subgoal_marker_.color.b = 0.6;
  robot_subgoal_marker_.scale.x = 0.3;
  robot_subgoal_marker_.scale.y = 0.3;

  // workspace marker
	workspace_marker_.header.frame_id = this->fixed_frame;
	workspace_marker_.ns = "scene";
	workspace_marker_.type = visualization_msgs::Marker::LINE_LIST;
	workspace_marker_.action = visualization_msgs::Marker::ADD;
	workspace_marker_.lifetime = ros::Duration(1.3f);
	workspace_marker_.scale.x = 0.05;
	workspace_marker_.color.a = 0.8;

	//planning marker. Calculated robot trajectories in space x time coordinates
	planning_marker_.header.frame_id = this->fixed_frame;
	planning_marker_.ns = "p3";
	planning_marker_.type = visualization_msgs::Marker::LINE_LIST;
	planning_marker_.action = visualization_msgs::Marker::ADD;
	planning_marker_.lifetime = ros::Duration(1.3f);
	planning_marker_.id = 0;
	planning_marker_.scale.x = 0.05;
	planning_marker_.color.a = 0.3;
	planning_marker_.color.r = 0.1;
	planning_marker_.color.g = 0.2;
	planning_marker_.color.b = 0.9;
  planning_marker_.points.reserve( this->planner_.get_number_of_vertex(  ) );


  //cylinder marker, planning random goals
	planning_goals_marker_.header.frame_id = this->fixed_frame;
	planning_goals_marker_.ns = "goals";
	planning_goals_marker_.type = visualization_msgs::Marker::CYLINDER;
	planning_goals_marker_.action = visualization_msgs::Marker::ADD;
	planning_goals_marker_.lifetime = ros::Duration(1.3f);
	planning_goals_marker_.color.a = 0.6;
	planning_goals_marker_.color.r = 0.9;
	planning_goals_marker_.color.g = 0.1;
	planning_goals_marker_.color.b = 0.0;
  planning_goals_marker_.scale.x = 0.2;
  planning_goals_marker_.scale.y = 0.2;
	planning_goals_marker_.scale.z = 0.1;
  planning_goals_marker_.pose.position.z = 0.05;
  planning_goals_marker_.pose.orientation.w = 1.0;

	
	//best path marker
  best_path_marker_.header.frame_id = this->fixed_frame;
	best_path_marker_.ns = "best3";
	best_path_marker_.type = visualization_msgs::Marker::LINE_STRIP;
	best_path_marker_.action = visualization_msgs::Marker::ADD;
	best_path_marker_.lifetime = ros::Duration(1.3f);
	best_path_marker_.scale.x = 0.05;
	best_path_marker_.color.a = 0.7;
	best_path_marker_.color.r = 0.9;
	best_path_marker_.color.g = 0.2;
	best_path_marker_.color.b = 0.2;
	
	//best path marker
	best_path2d_marker_ = best_path_marker_;
	best_path2d_marker_.ns = "best2";
  best_path2d_marker_.id = 0;

  //non-dominated marker
  nd_path_marker_.header.frame_id = this->fixed_frame;
	nd_path_marker_.ns = "nd3";
	nd_path_marker_.type = visualization_msgs::Marker::LINE_LIST;
	nd_path_marker_.action = visualization_msgs::Marker::ADD;
	nd_path_marker_.lifetime = ros::Duration(1.3f);
  nd_path_marker_.id = 0;
	nd_path_marker_.scale.x = 0.05;
	nd_path_marker_.color.a = 0.7;
	nd_path_marker_.color.r = 1.0;
	nd_path_marker_.color.g = 0.5;
	nd_path_marker_.color.b = 0.0;
	
	//non dominated path marker
	nd_path2d_marker_ = nd_path_marker_;
	nd_path2d_marker_.ns = "nd2";
  nd_path2d_marker_.id = 0;

  //laser obstacle marker
	laser_obstacle_marker_.header.frame_id = this->fixed_frame;
	laser_obstacle_marker_.ns = "obstacles";
	laser_obstacle_marker_.type = visualization_msgs::Marker::CYLINDER;
	laser_obstacle_marker_.action = visualization_msgs::Marker::ADD;
	laser_obstacle_marker_.lifetime = ros::Duration(1.3f);
	laser_obstacle_marker_.color.a = 0.7;
	laser_obstacle_marker_.color.r = 0.0;
	laser_obstacle_marker_.color.g = 0.0;
	laser_obstacle_marker_.color.b = 0.0;
  laser_obstacle_marker_.scale.x = 0.5;
  laser_obstacle_marker_.scale.y = 0.5;
	laser_obstacle_marker_.scale.z = 0.2;
  laser_obstacle_marker_.pose.position.z = 0.1;
  laser_obstacle_marker_.pose.orientation.w = 1.0;

	//force marker, a summation of all forces (red)
	force_marker_.header.frame_id = this->fixed_frame;
	force_marker_.ns =  "forces";
	force_marker_.type = visualization_msgs::Marker::ARROW;
	force_marker_.action = visualization_msgs::Marker::ADD;
	force_marker_.lifetime = ros::Duration(1.3f);
	force_marker_.scale.x = 0.2;
	force_marker_.scale.y = 0.25;
	force_marker_.color.a = 0.8;
	force_marker_.color.r = 1.0;
	force_marker_.color.g = 0.0;
	force_marker_.color.b = 0.0;
  force_marker_.points.push_back(  geometry_msgs::Point() );
  force_marker_.points.push_back(  geometry_msgs::Point() );

  // marker to goal (blue)
  force_goal_marker_ = force_marker_;
	force_goal_marker_.color.r = 0.0;
	force_goal_marker_.color.g = 0.4;
	force_goal_marker_.color.b = 1.0;

  // marker interaction with persons (green)
  force_int_person_marker_= force_marker_;
	force_int_person_marker_.color.r = 0.2;
	force_int_person_marker_.color.g = 0.85;
	force_int_person_marker_.color.b = 0.2;

  //marker dur to obstacles in the scene (black)
  force_obstacle_marker_ = force_marker_;
	force_obstacle_marker_.color.r = 0.0;
	force_obstacle_marker_.color.g = 0.0;
	force_obstacle_marker_.color.b = 0.0;

  // force of interaction with robot (purple)
  force_int_robot_marker_ = force_marker_;
  force_int_robot_marker_.color.r = 0.26;
  force_int_robot_marker_.color.g = 0.0;
  force_int_robot_marker_.color.b = 0.66;
  
  //text marker
	text_marker_.header.frame_id = this->fixed_frame;
	text_marker_.ns = "text";
	text_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	text_marker_.action = visualization_msgs::Marker::ADD;
	text_marker_.lifetime = ros::Duration(1.3f);
	text_marker_.scale.z = 0.3;
  text_marker_.pose.position.z = 0.0;
  text_marker_.color.a = 1.0;

}

void AkpLocalPlanner::fill_my_covariance_marker( visualization_msgs::Marker& marker, const SpointV_cov& point )
{
  marker.pose.position.x = point.x;
  marker.pose.position.y = point.y;
  marker.pose.position.z = 0.0;

  //plot covariances ellipses
  Eigen::Matrix2d cov;
  cov(0, 0) = point.cov.at(0);//xx
  cov(0, 1) = point.cov.at(1);
  cov(1, 0) = point.cov.at(4);
  cov(1, 1) = point.cov.at(5);//yy
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);
  //eig.compute(cov);
  marker.scale.x = sqrt( eig.eigenvalues()[0] ) + 0.001;//major vap
  marker.scale.y = sqrt( eig.eigenvalues()[1] ) + 0.001;//minor vap
  double angle = atan2( eig.eigenvectors()(1,0), eig.eigenvectors()(0,0) );
  marker.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
  
}

void AkpLocalPlanner::fill_forces_markers()
{
  const std::list<Cperson_abstract *>* person_list = planner_.get_scene( );
  Sforce force_to_goal, force_int_person , force_int_robot, force_obstacle, force_total;
  geometry_msgs::Point ros_point, ros_point_ini;
  unsigned int cont_f = 0;

  for( std::list<Cperson_abstract*>::const_iterator iit = person_list->begin();
          iit!=person_list->end(); iit++ )
  {
    // fill FORCES for each person
    //Sforce get_forces_person( Sforce&  force_to_goal, Sforce& force_int_person , Sforce& force_int_robot, Sforce& force_obstacle );
    force_total = (*iit)->get_forces_person( force_to_goal, force_int_person , force_int_robot, force_obstacle );
	  ros_point_ini.x = (*iit)->get_current_pointV().x;
	  ros_point_ini.y = (*iit)->get_current_pointV().y;

    //scaled person interaction force (green)
    force_int_person_marker_.points[0]= ros_point_ini;
    ros_point.x = ros_point_ini.x + force_int_person.fx;
    ros_point.y = ros_point_ini.y + force_int_person.fy;
    force_int_person_marker_.points[1] = ros_point;
    force_int_person_marker_.id = cont_f;
    ++cont_f;
    MarkerArray_msg_.markers.push_back(  force_int_person_marker_  );

    // robot to person interaction force (pink)
    force_int_robot_marker_.points[0]= ros_point_ini;
    ros_point.x = ros_point_ini.x + force_int_robot.fx;
    ros_point.y = ros_point_ini.y + force_int_robot.fy;
    force_int_robot_marker_.points[1] = ros_point;
    force_int_robot_marker_.id = cont_f;
    ++cont_f;
    MarkerArray_msg_.markers.push_back(  force_int_robot_marker_  );

    //map obstacles interaction force (black)
    force_obstacle_marker_.points[0] = ros_point_ini;
    ros_point.x = ros_point_ini.x + force_obstacle.fx;
    ros_point.y = ros_point_ini.y + force_obstacle.fy;
    force_obstacle_marker_.points[1] =  ros_point;
    force_obstacle_marker_.id = cont_f;
    ++cont_f;
    MarkerArray_msg_.markers.push_back(  force_obstacle_marker_  );

  }
  //print robot forces
  force_total = planner_.get_robot()->get_forces_person( force_to_goal, force_int_person , force_int_robot, force_obstacle );
  ros_point_ini.x = planner_.get_robot()->get_current_pointV().x;
  ros_point_ini.y = planner_.get_robot()->get_current_pointV().y;

  //scaled force to goal: ( blue )
  force_goal_marker_.points[0]= ros_point_ini;
  ros_point.x = ros_point_ini.x + force_to_goal.fx;
  ros_point.y = ros_point_ini.y + force_to_goal.fy;
  force_goal_marker_.points[1] = ros_point;
  force_goal_marker_.id = cont_f;
  ++cont_f;
  MarkerArray_msg_.markers.push_back(  force_goal_marker_  );

  //scaled person interaction force (green)
  force_int_person_marker_.points[0]= ros_point_ini;
  ros_point.x = ros_point_ini.x + force_int_person.fx;
  ros_point.y = ros_point_ini.y + force_int_person.fy;
  force_int_person_marker_.points[1] = ros_point;
  force_int_person_marker_.id = cont_f;
  ++cont_f;
  MarkerArray_msg_.markers.push_back(  force_int_person_marker_  );

  //map obstacles interaction force (black)
  force_obstacle_marker_.points[0] = ros_point_ini;
  ros_point.x = ros_point_ini.x + force_obstacle.fx;
  ros_point.y = ros_point_ini.y + force_obstacle.fy;
  force_obstacle_marker_.points[1] =  ros_point;
  force_obstacle_marker_.id = cont_f;
  ++cont_f;
  MarkerArray_msg_.markers.push_back(  force_obstacle_marker_  );

  //weighted resultant force (red)
  force_marker_.points[0] = ros_point_ini;
  ros_point.x = ros_point_ini.x + force_total.fx;
  ros_point.y = ros_point_ini.y + force_total.fy;
  force_marker_.points[1] =  ros_point;
  force_marker_.id = cont_f;
  ++cont_f;
  MarkerArray_msg_.markers.push_back(  force_marker_  );

}

void AkpLocalPlanner::fill_scene_markers()
{

  unsigned int cont(0);
  const std::vector<Sdestination>* dest = planner_.get_destinations();
  for( unsigned int i = 0; i < dest->size(); ++i)
  {
    cylinder_marker_.pose.position.x = dest->at(i).x;
    cylinder_marker_.pose.position.y = dest->at(i).y;
    cylinder_marker_.id = cont;
    MarkerArray_msg_.markers.push_back( cylinder_marker_ );
    ++cont;
  } 

  //draw robot destination, both local and global
  robot_goal_marker_.pose.position.x = planner_.get_robot_local_goal().x;
  robot_goal_marker_.pose.position.y = planner_.get_robot_local_goal().y;
  robot_goal_marker_.id = cont;
  ++cont;
  MarkerArray_msg_.markers.push_back( robot_goal_marker_ );
  robot_goal_marker_.pose.position.x = planner_.get_robot_goal().x;
  robot_goal_marker_.pose.position.y = planner_.get_robot_goal().y;
  robot_goal_marker_.id = cont;
  ++cont;
  MarkerArray_msg_.markers.push_back( robot_goal_marker_ );
  
  //if slicing mode, plot the set of destinations
  if( goal_providing_mode_ == AkpLocalPlanner::Slicing_global )
  {
    for( unsigned int i = 0; i< sliced_global_plan_.size(); ++i )
    {
      robot_subgoal_marker_.pose.position.x = sliced_global_plan_[i].pose.position.x;
      robot_subgoal_marker_.pose.position.y = sliced_global_plan_[i].pose.position.y;
      robot_subgoal_marker_.id = cont;
      ++cont;
      MarkerArray_msg_.markers.push_back( robot_subgoal_marker_ );
    }
  }

  //draw workspace of the robot
  double r = planner_.get_workspace_radii();
	geometry_msgs::Point ros_point;
  workspace_marker_.points.clear();
  workspace_marker_.id = cont;
  ++cont;
  for( unsigned int i = 0; i < 40; ++i )
  {
    ros_point.x = robot_pose_.x + r*cos( 2*i*PI/40 );
    ros_point.y = robot_pose_.y + r*sin( 2*i*PI/40 );
    workspace_marker_.points.push_back( ros_point ); 
    ros_point.x = robot_pose_.x + r*cos( (2*i+1)*PI/40 );
    ros_point.y = robot_pose_.y + r*sin( (2*i+1)*PI/40 );
    workspace_marker_.points.push_back( ros_point ); 
  }
  MarkerArray_msg_.markers.push_back( workspace_marker_ );
}

void AkpLocalPlanner::fill_planning_markers_2d()
{
  const std::vector<Spose>* plans = planner_.get_robot()->get_robot_planning_trajectory();
	geometry_msgs::Point ros_point;
  //random goals printing -------------------------------------------------------------------
  const std::vector<Spoint>* goals =  planner_.get_random_goals();
  for( unsigned int i = 0; i< goals->size(); ++i )
  {
    planning_goals_marker_.pose.position.x = goals->at(i).x;
    planning_goals_marker_.pose.position.y = goals->at(i).y;
    planning_goals_marker_.id = i;
    MarkerArray_msg_.markers.push_back( planning_goals_marker_ );
  }
  
  // plot non-dominated solutions in 2d ..................-------------------------------------
  const std::vector<unsigned int>* nondominated_path_index = planner_.get_robot_nondominated_plan_index();
  const std::vector<unsigned int>* nondominated_end_of_path_index = planner_.get_robot_nondominated_end_of_plan_index();
  nd_path2d_marker_.points.clear();
  //ROS_INFO("size = %d" , nondominated_path_index->size());
  for( unsigned int i = 0; i< nondominated_path_index->size(); i++ )
  {
    ros_point.x = plans->at(nondominated_path_index->at(i)).x;
    ros_point.y = plans->at(nondominated_path_index->at(i)).y;
    ros_point.z = 0.0;
    nd_path2d_marker_.points.push_back( ros_point );
  }
  //paths always even
  MarkerArray_msg_.markers.push_back( nd_path2d_marker_ );
  
  // plot text id of paths, if MultiObjective function mode is activated ---------------------
  text_marker_.id = 0;
  unsigned int i = 0;
  for(  ; i< nondominated_end_of_path_index->size(); i++ )
  {
    text_marker_.id++; 
    text_marker_.pose.position.x = plans->at(nondominated_end_of_path_index->at(i)).x;
    text_marker_.pose.position.y = plans->at(nondominated_end_of_path_index->at(i)).y;
    std::stringstream idText;
    idText << nondominated_end_of_path_index->at(i);
    text_marker_.text = idText.str();
    MarkerArray_msg_.markers.push_back( text_marker_ );
  }
  unsigned int text_markers_size = i;
	text_marker_.action = visualization_msgs::Marker::DELETE;
  for( ; i< text_markers_old_size_ ; i++)
  {
    text_marker_.id++;
    MarkerArray_msg_.markers.push_back( text_marker_ );
  }
	text_marker_.action = visualization_msgs::Marker::ADD;
  text_markers_old_size_ = text_markers_size;

}
void AkpLocalPlanner::fill_planning_markers_3d()
{
  const std::vector<Spose>* plans = planner_.get_robot()->get_robot_planning_trajectory();
  const std::vector<Sedge_tree>* edges = planner_.get_plan_edges();
	geometry_msgs::Point ros_point;
  double ini_time = plans->front().time_stamp;
	
	// plot 3d plan, complete tree
  planning_marker_.points.clear();
  for( unsigned int i = 1; i< plans->size(); ++i )
  {
    //lines segments beetwen 2 points
    ros_point.x = plans->at(i).x;
    ros_point.y = plans->at(i).y;
    ros_point.z = plans->at(i).time_stamp - ini_time;
    planning_marker_.points.push_back( ros_point );
    //ROS_INFO("message of size %d, at %d and parent %d", plans->size(), i, edges->at(i).parent);
    ros_point.x = plans->at( edges->at(i).parent ).x;
    ros_point.y = plans->at( edges->at(i).parent ).y;
    ros_point.z = plans->at(edges->at(i).parent).time_stamp - ini_time;
    planning_marker_.points.push_back( ros_point );
  }
  MarkerArray_msg_.markers.push_back( planning_marker_ );



  
    
  //plot best trajectory path in 3d
  const std::vector<unsigned int>* best_path_index = planner_.get_robot_plan_index();
  best_path_marker_.points.clear();
  //ROS_INFO("size = %d" , best_path_marker_.points.size());
  for( unsigned int i = 0; i< best_path_index->size(); i++ )
  {
    ros_point.x = plans->at(best_path_index->at(i)).x;
    ros_point.y = plans->at(best_path_index->at(i)).y;
    ros_point.z = plans->at(best_path_index->at(i)).time_stamp - ini_time;
    best_path_marker_.points.push_back( ros_point );
  }
  MarkerArray_msg_.markers.push_back( best_path_marker_ );
  
  // plot non-dominated solutions in 3d
  const std::vector<unsigned int>* nondominated_path_index = planner_.get_robot_nondominated_plan_index();
  nd_path_marker_.points.clear();
  //ROS_INFO("size = %d" , nondominated_path_index->size());
  for( unsigned int i = 0; i< nondominated_path_index->size(); i++ )
  {
    ros_point.x = plans->at(nondominated_path_index->at(i)).x;
    ros_point.y = plans->at(nondominated_path_index->at(i)).y;
    ros_point.z = plans->at(nondominated_path_index->at(i)).time_stamp - ini_time;
    nd_path_marker_.points.push_back( ros_point );
  }
  //paths always even
  MarkerArray_msg_.markers.push_back( nd_path_marker_ );


}

void AkpLocalPlanner::fill_people_prediction_markers_2d()
{
  //plot people trajectories 2d
  const std::vector<unsigned int>* best_path_index = planner_.get_robot_plan_index();
  const std::list<Cperson_abstract *>* person_list = planner_.get_scene( );
  const std::vector<SpointV_cov>* traj;
  unsigned int cont_2d = 0;
  for( std::list<Cperson_abstract*>::const_iterator iit = person_list->begin();
        iit!=person_list->end(); iit++ )
  {
	  traj = (*iit)->get_planning_trajectory();
    if ( traj->size() > best_path_index->size() )
    //if ( traj->empty() )
    {
      for( unsigned int i = 0; i< best_path_index->size(); ++i , ++cont_2d)
      {
        fill_my_covariance_marker( pred_traj2d_marker_,  traj->at(best_path_index->at(i)) );
        pred_traj2d_marker_.id = cont_2d;
        MarkerArray_msg_.markers.push_back( pred_traj2d_marker_ );
      }
    }
  }

}

void AkpLocalPlanner::fill_people_prediction_markers_3d()
{

	  const std::list<Cperson_abstract *>* person_list = planner_.get_scene( );
    const std::vector<SpointV_cov>* traj;
    unsigned int cont = 0;
    double time_stamp_ini = planner_.get_time();


    //ROS_INFO( "person list %d" , person_list->size() );
    for( std::list<Cperson_abstract*>::const_iterator iit = person_list->begin();
          iit!=person_list->end(); iit++ )
	  {
      if( (*iit)->get_planning_trajectory()->size() <= 1 )
  		  traj = (*iit)->get_prediction_trajectory();
      else
        traj = (*iit)->get_planning_trajectory();
      //fill prediction covariances
      for( unsigned int i = 0; i< traj->size(); ++i , ++cont)
      {
        fill_my_covariance_marker( pred_traj_marker_,  traj->at(i) );
        pred_traj_marker_.pose.position.z = traj->at(i).time_stamp - time_stamp_ini;//2d cov elevated the time azis(z)
        pred_traj_marker_.id = cont;
        MarkerArray_msg_.markers.push_back( pred_traj_marker_ );
      }


    }


}

void AkpLocalPlanner::fill_laser_obstacles()
{
  unsigned int cont(0);
  const std::vector<Spoint>* obstacles = planner_.get_laser_obstacles();
  for( unsigned int i = 0; i < obstacles->size(); ++i)
  {
    laser_obstacle_marker_.pose.position.x = obstacles->at(i).x;
    laser_obstacle_marker_.pose.position.y = obstacles->at(i).y;
    laser_obstacle_marker_.id = cont;
    ++cont;
    MarkerArray_msg_.markers.push_back( laser_obstacle_marker_ );
  }

}

void AkpLocalPlanner::fill_best_path_2d()
{
	geometry_msgs::Point ros_point;
  best_path2d_marker_.points.clear();
  const std::vector<Spose>* plans = planner_.get_robot()->get_robot_planning_trajectory();
  const std::vector<unsigned int>* best_path_index = planner_.get_robot_plan_index();
  //const std::vector<Spose>* best_plan = planner_.get_best_planning_trajectory();

  //ROS_ERROR("best path size = %d, and robot plan size = %d" , best_path_index->size(), plans->size()  );
  for( unsigned int i = 0; i< best_path_index->size(); i++ )
  {
    //ROS_INFO ( "size best_path = %d, and size of plans %d ", best_path_index->size(), plans->size()  );
    //ROS_INFO("filling %d, and best path index = %d" , i, best_path_index->at(i));
    ros_point.x = plans->at(best_path_index->at(i)).x;
    ros_point.y = plans->at(best_path_index->at(i)).y;
    ros_point.z = 0.0;
    best_path2d_marker_.points.push_back( ros_point );
  }

  MarkerArray_msg_.markers.push_back( best_path2d_marker_ );
}

