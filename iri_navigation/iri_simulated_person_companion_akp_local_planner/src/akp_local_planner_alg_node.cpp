#include "akp_local_planner_alg_node.h"
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
  planner_(5.0,500, Cplan_local_nav::F_RRT_GC_alpha),  // horitzont time=5.0, max_iter=500;
  costmap_ros_(NULL), 
  tf_(NULL), 
  group_simulation_now_(true),
  initialized_(false), 
  setup_(false), 
  vis_mode_(1),
  frozen_mode_(false), 
  move_base(true),
  text_markers_old_size_(0),
  slicing_path_diff_orientation_(20.0),
  move_base_client_("move_base", true), // add to send goal
  robot_ini_pose_x_(0.0),
  robot_ini_pose_y_(-15.0),
  robot_ini_pose_theta_(1.57),
  debug_antes_subgoals_entre_AKP_goals_(false),
  external_goal_(true),
  debug_real_test_companion_(false),
  debug_real_test_companion_robot_(false),
  iter(0),  
  // ini initial variables Cscen_sim
  n_persons_(0),
  simulation_mode_( AkpLocalPlanner::Normal ),
  ini_companion_(true),
  debug_person_companion_(false),
  debug_forces_akp_person_companion_(false),
  debug_goal_person_(false),
  distance_to_aproach_person_goal_(0.75), // min distance to aproach to the person goal
  we_habe_tracks_of_other_person_companion_(false),
  we_habe_destinations_of_other_person_companion_(false)
{
  pthread_mutex_init(&this->planner_mutex_,NULL);
  pthread_mutex_init(&this->fscan_mutex_,NULL);   
  pthread_mutex_init(&this->rscan_mutex_,NULL);
  pthread_mutex_init(&this->tracks_mutex_,NULL);
  pthread_mutex_init(&this->odom_mutex_,NULL);
  pthread_mutex_init(&this->odom_tibi_mutex_,NULL);
  pthread_mutex_init(&this->odom_other_person_comp_of_group_mutex_,NULL);
  pthread_mutex_init(&this->params_values_mutex_,NULL);
  pthread_mutex_init(&this->other_person_companion_tracks_mutex_,NULL);
  pthread_mutex_init(&this->other_person_destination_tracks_mutex_,NULL);
   // pthread_mutex_init(&this->cmd_vel_stop_mutex_,NULL);

 //this->loop_rate_ = 10;//in [Hz]

  //init class attributes if necessary
  this->isMoveBaseActive=false;
  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
  init();
  id_person_companion_=1;  // todo: meter en config del ini!!!

  //init class attributes if necessary
  this->scene_.set_bool_sim(true); // son de Csece_sim
  // this->loop_rate_ = 10;//in [Hz] => no se si en el plugin de navegacion se puede sear esto del Cscene_sim
 
  this->good_goal_=true;
  this->isMoveBaseActive=false;

  // change initial position
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(robot_ini_pose_x_, robot_ini_pose_y_, 0.0) );
  //ROS_INFO("(ROS) AkpLocalPlanner::fill_robot_companion_markers:  transform.pose.position.x=%f",transform.pose.position.x);
  //ROS_INFO("(ROS) AkpLocalPlanner::fill_robot_companion_markers: transform.pose.position.y=%f", transform.pose.position.y);   
  //ROS_INFO("(ROS) AkpLocalPlanner:: 1");     
  transform.setRotation( tf::Quaternion(0, 0, robot_ini_pose_theta_) ); // msg->theta= buscar y guardar por arriba orientación robot.
  //ROS_INFO("(ROS) AkpLocalPlanner:: 2");     
  //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/dabo2/base_footprint"));//br.sendTransform(tf::StampedTransfor
  std::string tf_name;
  //tf_name="/"+this->robot_companion_person_name_+"/base_footprint";
  tf_name="/"+this->robot_companion_person_name_+"/base_footprint";

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", tf_name));//br.sendTransform(tf::StampedTransfor

  // stop to evaluate costs:
  this->current_state = HSQ_IT;

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

void AkpLocalPlanner::odom_tibi_mutex_enter(void){
  pthread_mutex_lock(&this->odom_tibi_mutex_);
}
void AkpLocalPlanner::odom_tibi_mutex_exit(void){
  pthread_mutex_unlock(&this->odom_tibi_mutex_);
}

void AkpLocalPlanner::odom_other_person_comp_of_group_mutex_enter(void){
  pthread_mutex_lock(&this->odom_other_person_comp_of_group_mutex_);
}
void AkpLocalPlanner::odom_other_person_comp_of_group_mutex_exit(void){
  pthread_mutex_unlock(&this->odom_other_person_comp_of_group_mutex_);
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


/*
void AkpLocalPlanner::status_init_mutex_enter(void){
  pthread_mutex_lock(&this->status_init_mutex_);
}
void AkpLocalPlanner::status_init_mutex_exit(void){
  pthread_mutex_unlock(&this->status_init_mutex_);
}*/

/*void AkpLocalPlanner::cmd_vel_stop_mutex_enter(void){
  pthread_mutex_lock(&this->cmd_vel_stop_mutex_);
}
void AkpLocalPlanner::cmd_vel_stop_mutex_exit(void){
  pthread_mutex_unlock(&this->cmd_vel_stop_mutex_);
}*/

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
  pthread_mutex_destroy(&this->odom_tibi_mutex_);
  pthread_mutex_destroy(&this->params_values_mutex_);
  pthread_mutex_destroy(&this->other_person_companion_tracks_mutex_);
  pthread_mutex_destroy(&this->other_person_destination_tracks_mutex_);
 // pthread_mutex_destroy(&this->cmd_vel_stop_mutex_);
}

void AkpLocalPlanner::init()
{

  /* Publishers */
  this->cmd_vel_publisher_ = this->public_node_handle_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
 /* this->markers_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("planner_markers", 100);
  this->markers_publisher_m2_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("planner_markers_m2", 100);
  this->markers_publisher_comp_markers_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("planner_markers_comp_markers", 100);
  this->markers_publisher_people_prediction_time_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("planner_markers_people_predictions", 100);
*/
  this->cost_params_publisher_ = this->public_node_handle_.advertise<std_msgs::Float64MultiArray>("cost_values", 10);

  // Cscene_sim publishers
  this->tracksMarkers_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("tracksMarkers", 100);
  this->tracks_publisher_ = this->public_node_handle_.advertise<iri_perception_msgs::detectionArray>("tracks", 100);
  //pthread_mutex_init(&this->reset_mutex_,NULL);
  this->personCompanionAkpMarkers_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("person_companion_akp_planner_markers", 100);
  this->destinations_of_tracks_publisher_ = this->public_node_handle_.advertise<iri_perception_msgs::detectionArray>("destinations_of_tracks", 100);
 

 /* Subscribers */
  this->fscan_subscriber_   = this->public_node_handle_.subscribe<sensor_msgs::LaserScan>("front_scan" , 1, boost::bind(&AkpLocalPlanner::fscan_callback, this, _1));
  this->rscan_subscriber_   = this->public_node_handle_.subscribe<sensor_msgs::LaserScan>("rear_scan" , 1,  boost::bind(&AkpLocalPlanner::rscan_callback, this, _1));
  this->odom_subscriber_   = this->public_node_handle_.subscribe<nav_msgs::Odometry>("odom"  , 1, boost::bind(&AkpLocalPlanner::odom_callback, this, _1));
 this->odom_subscriber_tibi_   = this->public_node_handle_.subscribe<nav_msgs::Odometry>("odom_tibi"  , 1, boost::bind(&AkpLocalPlanner::odom_callback_tibi_robot, this, _1));
 this->odom_subscriber_other_person_comp_of_group_   = this->public_node_handle_.subscribe<nav_msgs::Odometry>("odom_other_person_of_group"  , 1, boost::bind(&AkpLocalPlanner::odom_callback_other_person_comp_of_group, this, _1));

//  this->tracks_subscriber_ = this->public_node_handle_.subscribe<iri_perception_msgs::detectionArray>("tracks", 1, boost::bind(&AkpLocalPlanner::tracks_callback, this, _1));
  this->params_values_subscriber_ = this->public_node_handle_.subscribe<std_msgs::Float64MultiArray>("params", 1, boost::bind(&AkpLocalPlanner::params_values_callback, this, _1));


  this->other_person_companion_tracks_subscriber_ = this->public_node_handle_.subscribe("other_person_companion_tracks", 100, &AkpLocalPlanner::other_person_companion_tracks_callback, this);

  this->other_person_destination_tracks_subscriber_ = this->public_node_handle_.subscribe("other_person_destination_tracks", 100, &AkpLocalPlanner::other_person_destination_tracks_callback, this);

  //pthread_mutex_init(&this->other_person_companion_tracks_mutex_,NULL);

//  this->status_init_subscriber_ = this->public_node_handle_.subscribe<iri_perception_msgs::restartSim>("status_init", 1, boost::bind(&AkpLocalPlanner::status_init_callback, this, _1));


      
  Float64_msg_.data.resize(8,0.0);//TODO cuantos elementos?


  // To stop code for evaluate the costs.
  this->cmd_vel_stop_subscriber_ = this->public_node_handle_.subscribe("cmd_vel_stop", 1, &AkpLocalPlanner::cmd_vel_stop_callback, this);


  // [init services] ==> Csene sim
  this->reset_server_ = this->public_node_handle_.advertiseService("reset", &AkpLocalPlanner::resetCallback, this);

  // [init services] => My cervice reset cscene sim, persons to initial pose
  this->init_simulations_server_ = this->public_node_handle_.advertiseService("init_simulations", &AkpLocalPlanner::init_simulationsCallback, this);
  //pthread_mutex_init(&this->init_simulations_mutex_,NULL); 

 std::cout << " [DABOOOOOOOOOOOOOOOOOOOOO] out AkpLocalPlanner::init() " << std::endl;

} 

void AkpLocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  std::cout << " DABOOOOOOOOOOOOOOOOOOOOOOOOO AkpLocalPlanner::initialize DABO" << std::endl;
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
    private_nh.getParam("robot", this->robot_);  // tipo de robot, no pose inicial.
    double horizon_time, v_max, w_max, av_max, av_break, aw_max, platform_radii;
    double cost_distance,cost_orientation,cost_w_robot,cost_w_people,cost_time, cost_obs, cost_old_path, cost_l_minima;//, cost_companion; (NO TODO)
    int mode, nvertex;
    this->planner_.set_dt(0.2);//TODO depends upon the move_base frame_rate
    this->scene_.set_dt(0.2);
	  this->scene_.get_akp_for_person_companion()->set_dt(0.2);
	 /* if(group_simulation_now_){
	    this->scene_.get_akp_for_person_companion_2groupPers()->set_dt(0.2);
	  }*/
    //robot frames
    this->fixed_frame = "/map";
    //this->robot_frame  = "/" + this->robot_ + "/base_link";
    //this->robot_frame  = "/dabo2/base_link";  // TODO: setear el dabo2 o dabo desde fuera del nodo. 
    //this->robot_frame  ="/" + this->robot_companion_person_name_ + "/base_link";
    this->robot_frame  = "/" +this->robot_companion_person_name_ + "/base_link";  
	  ROS_INFO("IIIIIIIIIIIIIIIIII!!!! target_frame.c_str()=%s",this->robot_frame.c_str());
    private_nh.getParam("move_base", this->move_base);

    // si quisiera hackear el move_base, sería poner esto:
    this->move_base=true;  //ely! si quiero que al principio este parado hay que quitarlo
    //ROS_INFO("AkpLocalPlanner::initialize: this->move_base=",this->move_base);
    
    //if(debug_antes_subgoals_entre_AKP_goals_){
      std::cout << " AkpLocalPlanner::initialize this->move_base ="<< this->move_base  << std::endl;
    //}
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
    //private_nh.getParam("cost_companion",  cost_companion); //son parametros de planing, no los costes que salen fuera. NO necesito ninguno para companion!!! // NO TODO: add companion cost
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
    ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    ROS_INFO("AkpLocalPlanner::initialize: esfm_k=%f", params[0]);
    ROS_INFO("AkpLocalPlanner::initialize: esfm_d=%f", params[4]);
    ROS_INFO("AkpLocalPlanner::initialize: esfm_to_person_lambda=%f", params[1]);
    ROS_INFO("AkpLocalPlanner::initialize: esfm_to_person_A=%f", params[2]);
    ROS_INFO("AkpLocalPlanner::initialize: esfm_to_person_B=%f", params[3]);

    this->planner_.set_sfm_to_person( params );
    private_nh.getParam("esfm_to_robot_lambda",params[1] );
    private_nh.getParam("esfm_to_robot_A", params[2]);
    private_nh.getParam("esfm_to_robot_B", params[3]);
    ROS_INFO("AkpLocalPlanner::initialize: esfm_to_robot_lambda=%f", params[1]);
    ROS_INFO("AkpLocalPlanner::initialize: esfm_to_robot_A=%f", params[2]);
    ROS_INFO("AkpLocalPlanner::initialize: esfm_to_robot_B=%f", params[3]);

    planner_.set_sfm_to_robot( params );
    private_nh.getParam("esfm_to_obstacle_lambda",params[1] );
    private_nh.getParam("esfm_to_obstacle_A", params[2]);
    private_nh.getParam("esfm_to_obstacle_B", params[3]);
    ROS_INFO("AkpLocalPlanner::initialize: esfm_to_obstacle_lambda=%f", params[1]);
    ROS_INFO("AkpLocalPlanner::initialize: esfm_to_obstacle_A=%f", params[2]);
    ROS_INFO("AkpLocalPlanner::initialize: esfm_to_obstacle_B=%f", params[3]);
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


    /*
    esfm_to_person_companion_A: 5.05
    esfm_to_person_companion_B: 0.91
    esfm_to_person_companion_lambda: 0.25
    esfm_companion_d: 0.2*/
   
    private_nh.getParam("esfm_to_person_companion_lambda",params[1] );
    private_nh.getParam("esfm_to_person_companion_A", params[2]);
    private_nh.getParam("esfm_to_person_companion_B", params[3]);
    private_nh.getParam("esfm_companion_d", params[4]);
    ROS_INFO("(person_companion) AkpLocalPlanner::initialize: esfm_to_person_companion_lambda=%f", params[1]);
    ROS_INFO("AkpLocalPlanner::initialize: esfm_to_person_companion_A=%f", params[2]);
    ROS_INFO("AkpLocalPlanner::initialize: esfm_to_person_companion_B=%f", params[3]);
    ROS_INFO("AkpLocalPlanner::initialize: esfm_companion_d=%f", params[4]);
	
    this->planner_.set_sfm_to_person_companion( params );

    this->scene_.get_akp_for_person_companion()->set_sfm_to_person_companion( params );
    /*if(group_simulation_now_){
	    this->scene_.get_akp_for_person_companion_2groupPers()->set_sfm_to_person_companion( params );
    }*/
  

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
    
    //init_force_planner_and_markers();

    dsrv_ = new dynamic_reconfigure::Server<iri_simulated_person_companion_akp_local_planner::AkpLocalPlannerConfig>(private_nh);
    dynamic_reconfigure::Server<iri_simulated_person_companion_akp_local_planner::AkpLocalPlannerConfig>::CallbackType cb = boost::bind(&AkpLocalPlanner::reconfigureCallback, this, _1, _2);
    dsrv_->setCallback(cb);
    initialized_ = true;

    // Do initialization goal!
	ROS_INFO("AkpLocalPlanner::initialize: [DABO] antes GOAL en initialize()");
    this->actual_goal.x=this->robot_pose_.x;
    this->actual_goal.y=this->robot_pose_.y;
    this->doGoal();
    this->robot_pose_2personGroup_.x=this->robot_pose_.x+1.0;
    this->robot_pose_2personGroup_.y=this->robot_pose_.y+1.0;
    this->robot_pose_2personGroup_.theta=this->robot_pose_.theta;
    this->robot_pose_2personGroup_.time_stamp=this->robot_pose_.time_stamp;
    this->robot_pose_2personGroup_.v=this->robot_pose_.v;
    this->robot_pose_2personGroup_.w=this->robot_pose_.w;

    //ROS_INFO("AkpLocalPlanner::initialize: [DABO] Fin initialize()");

  }
  else
    ROS_WARN("AkpLocalPlanner::initialize: [DABO] This planner has already been initialized, you can't call it twice, doing nothing");

  // primer send a goal!!!(ely)= ya no computa, ya aparece en la posición que le toca.== Si se mueve de posicion inicial, descomentar esto.
  /*Spoint ini_goal(-1.0,-8.0,ros::Time::now().toSec());
  this->actual_goal=ini_goal;
  this->doGoal(this->actual_goal);  */

  /* Initialize Csecen_sim */
  init_sim();
  //this->planner_.update_scene_companion_simulation(this->obs,true); // initialice scene

}

bool AkpLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{ 


 if(debug_person_companion_){
    std::cout <<std::endl<<std::endl;
    ROS_INFO("[Person_companion DABOOOOOOOOOOOOOOOOOOOOO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
 }

  ros::Time now    = ros::Time::now();
  // (2) funcion, local planer. Da las velocidades del robot para seguir el plan.

  if(debug_antes_subgoals_entre_AKP_goals_){
    ROS_INFO("AkpLocalPlanner::computeVelocityCommands");
  }
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
    //ROS_INFO(" (planner) fscan_received");
    this->laser_points = this->scan2points(this->fscan);
    this->fscan_received=false;
  }
  this->fscan_mutex_exit();

  this->rscan_mutex_enter();
  if(this->rscan_received)
  {
    //ROS_INFO(" (planner) rscan_received");
    std::vector< Spoint > rear_points;
    rear_points = this->scan2points(this->rscan);
    this->laser_points.insert(this->laser_points.end(), rear_points.begin(), rear_points.end());
    this->rscan_received=false;
  }
  this->rscan_mutex_exit();

  //this->planner_.read_laser_scan( this->laser_points );

 //ROS_INFO("2 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");

  this->scene_.read_laser_scan_person_companion_akp_planner( this->laser_points, true );
  // ROS_INFO("3 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
  this->scene_.get_akp_for_person_companion()->read_laser_scan_person_companion_akp_planner( this->laser_points, true ); // incluir laser points en la person companion 1
  
  // enter in the library the tracks of the other person companion of the group
  if((we_habe_tracks_of_other_person_companion_)&&(group_simulation_now_)){
    this->scene_.set_tracks_other_person_companion_of_group(obs_other_person_companion_);
    we_habe_tracks_of_other_person_companion_=false;
  }
  
  
  if((we_habe_destinations_of_other_person_companion_)&&(group_simulation_now_)){
    this->scene_.set_destinationsOFtracks_other_person_companion_of_group(destinations_other_person_companion_);
    we_habe_destinations_of_other_person_companion_=false;
  }


 /* if(group_simulation_now_){
	this->scene_.get_akp_for_person_companion_2groupPers()->read_laser_scan_person_companion_akp_planner( this->laser_points, true ); // incluir laser points en la person companion 2
  }*/
  //ROS_INFO("4 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
  //update robot position
  /* this->odom_mutex_enter();
  this->planner_.update_robot(this->robot_pose_);
  this->odom_mutex_exit();*/
  double vx_a=this->robot_pose_.v*cos(this->robot_pose_.theta);
  double vy_a=this->robot_pose_.v*sin(this->robot_pose_.theta);	
  SpointV_cov person_point;
  person_point=SpointV_cov(this->robot_pose_.x,this->robot_pose_.y,this->robot_pose_.time_stamp,vx_a,vy_a);
  double vx_a2;
  double vy_a2;
  SpointV_cov person_point2;
  if(group_simulation_now_){
    vx_a2=this->robot_pose_2personGroup_.v*cos(this->robot_pose_2personGroup_.theta);
    vy_a2=this->robot_pose_2personGroup_.v*sin(this->robot_pose_2personGroup_.theta);	
    person_point2;
    person_point2=SpointV_cov(this->robot_pose_2personGroup_.x,this->robot_pose_2personGroup_.y,this->robot_pose_2personGroup_.time_stamp,vx_a2,vy_a2);
  }


  //ROS_INFO("5 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
  this->scene_.get_akp_for_person_companion()->update_person_companion(this->robot_pose_,person_point);
 /* if(group_simulation_now_){
    this->scene_.get_akp_for_person_companion_2groupPers()->update_person_companion(this->robot_pose_2personGroup_,person_point2);
  }*/
  //ROS_INFO("6 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
  this->scene_.set_initial_companion_person_robot_point(person_point);
  if(group_simulation_now_){
    this->scene_.set_initial_companion_person_robot_point_2personGroup(person_point2);
  } 
  //ROS_INFO("7 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
  this->scene_.set_initial_companion_person_robot_Spose(this->robot_pose_);
  if(group_simulation_now_){
     this->scene_.set_initial_companion_person_robot_Spose_2personGroup(this->robot_pose_2personGroup_);
  } 
  // ROS_INFO("8 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
  if(debug_goal_person_){
    ROS_INFO("(ROS) person_point:");
    person_point.print();
  }
  /* update simulated robot  [tibi=/tibi/base_link] TODO: poder cambiarla con el dinamic reconfigure*/
  //Get Robot position: transform empty/zero pose from base_link to map
  //Get Robot position: transform empty/zero pose from base_link to map
  std::string target_frame = "/map";
  /* //TODO set param in .launch  <param name="~/robot" type="string" value="$(env ROBOT)" />
  std::string source_frame2 ="/tibi/base_link"; 
  ros::Time target_time2    = ros::Time::now();


  bool tf_exists2 = tf_listener_.waitForTransform(target_frame, source_frame2, target_time2, ros::Duration(1), ros::Duration(0.01));
  if(tf_exists2)
  {
    geometry_msgs::PoseStamped poseIn;
    geometry_msgs::PoseStamped poseOut;
    poseIn.header.stamp    = target_time2;
    poseIn.header.frame_id = source_frame2;
    poseIn.pose.orientation.z = 1.0; //valid quaternion
    tf_listener_.transformPose(target_frame, poseIn, poseOut);
    //TODO add velocities in /map coordinates to the scene update
    scene_.update_robot(Spose(poseOut.pose.position.x, poseOut.pose.position.y, poseOut.header.stamp.toSec()));
  }
  else
  {
    ROS_INFO("fail, no transform found");
  }*/

  //TODO set param in .launch  <param name="~/robot" type="string" value="$(env ROBOT)" />
  //std::string source_frame = "/" + this->robot_ + "/base_link";  // Target frame del robot en simulacion!!!
 /* std::string act_robot_="tibi"; // TODO: comprobar que funciona.
  std::string source_frame = "/" + act_robot_ + "/base_link";  // Target frame del robot en simulacion!!!

  if(debug_person_companion_){
    ROS_INFO(" 6 IN AkpLocalPlanner::mainNodeThread");
  }
  ros::Time target_time    = ros::Time::now();
  //ROS_INFO("target_frame.c_str()=%s",target_frame.c_str());
  //ROS_INFO("target_frame.c_str()=%s",source_frame.c_str());
  bool tf_exists = tf_listener_.waitForTransform(target_frame, source_frame, target_time, ros::Duration(1), ros::Duration(0.01));
  //ROS_INFO("AkpLocalPlanner::mainNodeThread: antes if(tf_exists)"); 
  if(debug_person_companion_){ 
    ROS_INFO(" 7 IN AkpLocalPlanner::mainNodeThread");
  }

  if(tf_exists)
  {
    geometry_msgs::PoseStamped poseIn;
    geometry_msgs::PoseStamped poseOut;
    poseIn.header.stamp    = target_time;
    poseIn.header.frame_id = source_frame;
    poseIn.pose.orientation.z = 1.0; //valid quaternion
    tf_listener_.transformPose(target_frame, poseIn, poseOut);
    //TODO add velocities in /map coordinates to the scene update
    Crobot* person_companion_in=scene_.get_akp_for_person_companion()->get_person_companion_akp();
    SpointV_cov person_Spoint_in=person_companion_in->get_current_pointV();
    person_Spoint_in.time_stamp=poseOut.header.stamp.toSec();
    Spose person_Spose_in=person_companion_in->get_current_pose();
    person_Spose_in.time_stamp=poseOut.header.stamp.toSec();
    scene_.update_robot(Spose(poseOut.pose.position.x, poseOut.pose.position.y, poseOut.header.stamp.toSec()));
    scene_.get_akp_for_person_companion()->update_robot(Spose(poseOut.pose.position.x, poseOut.pose.position.y, poseOut.header.stamp.toSec()));
    //ROS_INFO("AkpLocalPlanner::mainNodeThread: if(tf_exists)");  
  }
  else
  {
    ROS_INFO("fail, no transform found");
  }*/
	
  //ROS_INFO("NEW IIIIIIIIIIIIIIIIII!!!! target_frame.c_str()=%s",this->robot_frame.c_str());
  /* update person companion [dabo=/dabo/base_link] TODO: poder cambiarla con el dinamic reconfigure */
   //std::string act_robot_="dabo2"; // TODO: comprobar que funciona. 
  std::string act_robot_=this->robot_companion_person_name_;
  ROS_INFO("7777777 robot_companion_person_name_out_node_=%s",robot_companion_person_name_.c_str());
  //std::string source_frame = "/" + act_robot_ + "/base_link";  // Target frame del robot en simulacion!!!
	std::string source_frame = "/" + act_robot_ + "/base_link";  // Target frame del robot en simulacion!!!
  if(debug_person_companion_){
    ROS_INFO(" 8 IN AkpLocalPlanner::mainNodeThread");
  }
  ros::Time  target_time    = ros::Time::now();
  //ROS_INFO("target_frame.c_str()=%s",target_frame.c_str());
  //ROS_INFO("7777777 IIII!!!!!!!!!!!!!!!!! source_frame.c_str()=%s",source_frame.c_str());
  bool tf_exists = tf_listener_.waitForTransform(target_frame, source_frame, target_time, ros::Duration(1), ros::Duration(0.01));
  //ROS_INFO("AkpLocalPlanner::mainNodeThread: antes if(tf_exists)"); 
  if(debug_person_companion_){ 
    ROS_INFO(" 9 IN AkpLocalPlanner::mainNodeThread");
  }

  geometry_msgs::PoseStamped poseIn;
  geometry_msgs::PoseStamped poseOut; // provisional, sacarlo fuera...

  if(tf_exists)
  {   
    poseIn.header.stamp    = target_time;
    poseIn.header.frame_id = source_frame;
    poseIn.pose.orientation.z = 1.0; //valid quaternion
    tf_listener_.transformPose(target_frame, poseIn, poseOut);
    //TODO add velocities in /map coordinates to the scene update
    Crobot* person_companion_in=scene_.get_akp_for_person_companion()->get_person_companion_akp();
    SpointV_cov person_Spoint_in=person_companion_in->get_current_pointV();
    person_Spoint_in.time_stamp=poseOut.header.stamp.toSec();
    Spose person_Spose_in=person_companion_in->get_current_pose();
    person_Spose_in.time_stamp=poseOut.header.stamp.toSec();
    //ROS_INFO("7777777 [Person_companion DABO- other person of group] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
    //ROS_INFO("7777777 [Person_companion DABO- other person of group] robot_person_X=%f, robot_person_Y=%f ",poseOut.pose.position.x,poseOut.pose.position.y);
    //scene_.get_akp_for_person_companion()->update_person_companion(Spose(poseOut.pose.position.x, poseOut.pose.position.y, poseOut.header.stamp.toSec()),SpointV_cov(poseOut.pose.position.x, poseOut.pose.position.y,      	poseOut.header.stamp.toSec()));
    scene_.update_person_companion(Spose(poseOut.pose.position.x, poseOut.pose.position.y, poseOut.header.stamp.toSec()),SpointV_cov(poseOut.pose.position.x, poseOut.pose.position.y,      	poseOut.header.stamp.toSec()));
    // TODO: mirar que no vaya mal al no incluir velocidades.
    //ROS_INFO("AkpLocalPlanner::mainNodeThread: if(tf_exists)");  
  }
  else
  {
    ROS_INFO("fail, no transform found");
  }


  /* INI update position person companion 2*/
  /*Crobot* person_companion_in2;
  SpointV_cov person_Spoint_in2;
  Spose person_Spose_in2;
  if(group_simulation_now_){
    person_companion_in2=scene_.get_akp_for_person_companion_2groupPers()->get_person_companion_akp();
    person_Spoint_in2=person_companion_in2->get_current_pointV();
    person_Spoint_in2.time_stamp=poseOut.header.stamp.toSec();
    person_Spose_in2=person_companion_in2->get_current_pose();
    person_Spose_in2.time_stamp=poseOut.header.stamp.toSec();
    scene_.get_akp_for_person_companion_2groupPers()->update_person_companion(person_Spose_in2,person_Spoint_in2);
    robot_pose_2personGroup_=person_Spose_in2;
  } */
 
/* FIN update position person companion 2*/

	// provisional para tibi. TODO: volver a poner el de arriba.
		//scene_.update_robot(Spose(0,0, poseOut.header.stamp.toSec()));
   // scene_.get_akp_for_person_companion()->update_robot(Spose(0, 0, poseOut.header.stamp.toSec()));
    // ROS_INFO("9 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
    //scene_.update_robot(this->robot_pose_tibi_);
 //ROS_INFO("10 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
   // scene_.get_akp_for_person_companion()->update_robot(this->robot_pose_tibi_);

//ROS_INFO("11 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
// UPDATE Scene for robot is not needed!
// ROS_INFO("11 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
// UPDATE Scene for robot is not needed!
/*
  this->we_have_companion_person_=false;
  //update people observations
  this->tracks_mutex_enter();
  //ROS_INFO("AkpLocalPlanner::computeVelocityCommands: (antes update scene) this->obs.size()=%d=",this->obs.size());
  
  if(debug_real_test_companion_){
    for(unsigned int u=0;u<this->obs.size();u++){
      ROS_INFO( "this->obs[%d].vx=%f; .vy=%f",u,this->obs[u].vx,this->obs[u].vy);
    }
  }

  this->planner_.update_scene(this->obs,this->we_have_companion_person_); 

  if(debug_real_test_companion_robot_){
    if(this->we_have_companion_person_){
      ROS_INFO( "1 (tiene que ser true) this->we_have_companion_person_=true");
    }else{
      ROS_INFO( "1 (tiene que ser true) this->we_have_companion_person_=false");
    }
  }

  //this->planner_.update_scene_companion_simulation(this->obs);
  //this->obs.clear();//when no updates are done, this message should be clear, and not the previous
  this->tracks_mutex_exit();
 */

// no compila bien.

  std::string target_frame2 = "/map";
//ROS_INFO("11.1 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
  //TODO set param in .launch  <param name="~/robot" type="string" value="$(env ROBOT)" />
//ROS_INFO("11.2 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
  std::string source_frame2 = "/tibi/base_link"; 
  ros::Time target_time2    = ros::Time::now();
//ROS_INFO("11.3 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
  bool tf_exists2 = tf_listener_.waitForTransform(target_frame2, source_frame2, target_time2, ros::Duration(1), ros::Duration(0.01));
//ROS_INFO( " antes tf_exist");
//ROS_INFO("11.4 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
  if(tf_exists2)
  {
//ROS_INFO("11.5 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
    geometry_msgs::PoseStamped poseIn2;
    geometry_msgs::PoseStamped poseOut2;
    poseIn2.header.stamp    = target_time2;
    poseIn2.header.frame_id = source_frame2;
    poseIn2.pose.orientation.z = 1.0; //valid quaternion
    tf_listener_.transformPose(target_frame2, poseIn2, poseOut2);
    //TODO add velocities in /map coordinates to the scene update
  //ROS_INFO("11.6 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
    scene_.update_robot(Spose(poseOut2.pose.position.x, poseOut2.pose.position.y, poseOut2.header.stamp.toSec()));
  //ROS_INFO("11.7 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
  scene_.get_akp_for_person_companion()->update_robot(Spose(poseOut2.pose.position.x, poseOut2.pose.position.y, poseOut2.header.stamp.toSec()));
 /* if(group_simulation_now_){
    scene_.get_akp_for_person_companion_2groupPers()->update_robot(Spose(poseOut2.pose.position.x, poseOut2.pose.position.y, poseOut2.header.stamp.toSec()));
  } */	


//ROS_INFO("11.8 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
	//ROS_INFO( " tibi.x=%f; tibi.y=%f;",poseOut2.pose.position.x,poseOut2.pose.position.y);

	//scene_.set_true_robot(Spose(poseOut.pose.position.x, poseOut.pose.position.y, poseOut.header.stamp.toSec()))
  }
  else
  {
    ROS_INFO("fail, no transform found");
 //ROS_INFO("fail, no transform found");
  }


// +++++ INI update position other person that accompanies the group:
if(group_simulation_now_){
    std::string target_frame3 = "/map";
    //ROS_INFO("11.1 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
    //TODO set param in .launch  <param name="~/robot" type="string" value="$(env ROBOT)" />
    //ROS_INFO("11.2 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
    // std::string source_frame2 = "/tibi/base_link"; 
    std::string act_robot2_=this->robot_companion_person_name_out_node_;
    ROS_INFO("7777777 (other perso) robot_companion_person_name_out_node_=%s",robot_companion_person_name_out_node_.c_str());
	  std::string source_frame3 = "/" + act_robot2_ + "/base_link"; // Target frame del robot en simulacion!!!

    ros::Time target_time3    = ros::Time::now();
    //ROS_INFO("11.3 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
    bool tf_exists3 = tf_listener_.waitForTransform(target_frame3, source_frame3, target_time3, ros::Duration(1), ros::Duration(0.01));
    //ROS_INFO( " antes tf_exist");
    //ROS_INFO("11.4 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
    if(tf_exists3)
    {
    //ROS_INFO("11.5 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
      geometry_msgs::PoseStamped poseIn3;
      geometry_msgs::PoseStamped poseOut3;
      poseIn3.header.stamp    = target_time3;
      poseIn3.header.frame_id = source_frame3;
      poseIn3.pose.orientation.z = 1.0; //valid quaternion
      tf_listener_.transformPose(target_frame3, poseIn3, poseOut3);
      //TODO add velocities in /map coordinates to the scene update
      // ROS_INFO("7777777 [Person_companion DABO- other person of group] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
      //ROS_INFO("7777777 [Person_companion DABO- other person of group] person_X=%f, person_Y=%f ",poseOut3.pose.position.x,poseOut3.pose.position.y);
	    double vx_rob2_comp=robot_pose_other_comp_pers_.v*cos(robot_pose_other_comp_pers_.theta);
      double vy_rob2_comp=robot_pose_other_comp_pers_.v*sin(robot_pose_other_comp_pers_.theta);
      //ROS_INFO("7777777 [Person_companion DABO- other person of group] person_VX=%f, person_VY=%f ",vx_rob2_comp,vy_rob2_comp);
	// robot_pose_other_comp_pers_.theta; robot_pose_other_comp_pers_.v
	// TODO: me falta vx e vy!?? creo que el robot las tiene...
      scene_.set_other_companion_person_out_odometry_spose(robot_pose_other_comp_pers_);
      scene_.set_other_companion_person_out_odometry_spoint(SpointV_cov(poseOut3.pose.position.x, poseOut3.pose.position.y, poseOut3.header.stamp.toSec(),vx_rob2_comp,vy_rob2_comp)); // double x_ , double y_, double time_stamp_, double vx_, double vy_
	    // scene_.update_robot(Spose(poseOut3.pose.position.x, poseOut3.pose.position.y, poseOut3.header.stamp.toSec()));
      // !!!!! scene_.update_other_people_of_group(Spose(poseOut3.pose.position.x, poseOut3.pose.position.y, poseOut3.header.stamp.toSec()));
      //ROS_INFO("11.7 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
      // scene_.get_akp_for_person_companion()->update_robot(Spose(poseOut2.pose.position.x, poseOut2.pose.position.y, poseOut2.header.stamp.toSec()));
      // !!!! scene_.get_akp_for_person_companion()->update_other_people_of_group(Spose(poseOut3.pose.position.x, poseOut3.pose.position.y,   poseOut3.header.stamp.toSec()));
     // if(group_simulation_now_){
      // mirar si hay que cambiarla.
     // scene_.get_akp_for_person_companion_2groupPers()->update_robot(Spose(poseOut3.pose.position.x, poseOut3.pose.position.y, poseOut3.header.stamp.toSec()));
    //} 	


    //ROS_INFO("11.8 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
	  //ROS_INFO( " tibi.x=%f; tibi.y=%f;",poseOut2.pose.position.x,poseOut2.pose.position.y);

	  //scene_.set_true_robot(Spose(poseOut.pose.position.x, poseOut.pose.position.y, poseOut.header.stamp.toSec()))
    }
    else
    {
      ROS_INFO("fail, no transform found");
   //ROS_INFO("fail, no transform found");
    }
}
// +++++ FIN update position other person that accompanies the group:

//ROS_INFO("11.9 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
  // update Cscene_sim
  double t = ros::Time::now().toSec();
  bool robot_plan_succed=false;
  Spose best_next_pose;
//ROS_INFO("11.10 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
  robot_plan_succed=do_Cscene_sim(best_next_pose);
//ROS_INFO("11.11 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
    // to stop the simulated person, to evaluate the costs.
  /*  if(this->current_state==HSQ_STOP){
      robot_plan_succed=false;
    }

    if(this->current_state==HSQ_IT){      
       this->current_state = HSQ_STOP;
    }*/
   // ROS_INFO("AkpLocalPlanner::computeVelocityCommands:this->robot_pose_ out");
   // this->robot_pose_.print(); 
//ROS_INFO("12.1 [Person_companion DABO] robot_plan_succed=true; move_base=true; this->robot_pose_.v=%f; this->robot_pose_.w=%f;",this->robot_pose_.v,this->robot_pose_.w);
/*if(robot_plan_succed){
	ROS_INFO("12.2 [Person_companion DABO] robot_plan_succed=true; robot_plan_succed=true; ");
}else{
	ROS_INFO("12.2 [Person_companion DABO] robot_plan_succed=true; robot_plan_succed=false; ");
}
if(this->move_base){
	ROS_INFO("12.2 [Person_companion DABO] robot_plan_succed=true; move_base=true; ");
}else{
	ROS_INFO("12.2 [Person_companion DABO] robot_plan_succed=true; move_base=false; ");
}*/

 //ROS_INFO("12.2 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!this->robot_pose_.v=%f; this->robot_pose_.w=%f;",this->robot_pose_.v,this->robot_pose_.w);
//Sdestination best_person_dest_act=this->scene_.get_person_companion_person_abstract().get_best_dest();
Sdestination best_person_dest_act=this->scene_.get_akp_for_person_companion()->get_person_companion_akp()->get_best_dest();
 //ROS_INFO("13 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!this->robot_pose_.v=%f; this->robot_pose_.w=%f;",this->robot_pose_.v,this->robot_pose_.w);
Spoint best_person_dest_spoin(best_person_dest_act.x,best_person_dest_act.y,ros::Time::now().toSec());
 //ROS_INFO("14 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!this->robot_pose_.v=%f; this->robot_pose_.w=%f;",this->robot_pose_.v,this->robot_pose_.w);
Spoint robot_position_act(this->robot_pose_.x,this->robot_pose_.y,ros::Time::now().toSec());
 //ROS_INFO("15 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!this->robot_pose_.v=%f; this->robot_pose_.w=%f;",this->robot_pose_.v,this->robot_pose_.w);
  if(robot_plan_succed){
 	//ROS_INFO("15 [Person_companion DABO] robot_plan_succed=true; move_base=true; this->robot_pose_.v=%f; this->robot_pose_.w=%f;",this->robot_pose_.v,this->robot_pose_.w);
      this->move_base=true;
  }else{
    this->robot_pose_.v=0.000000000001;
    this->robot_pose_.w=0;
    robot_plan_succed=true;
    this->move_base=false;
    best_next_pose=this->robot_pose_;
	//ROS_INFO("15 [Person_companion DABO] ELSE!!! vels=0!!! robot_plan_succed=true; move_base=true; this->robot_pose_.v=%f; this->robot_pose_.w=%f;",this->robot_pose_.v,this->robot_pose_.w);
  }
//ROS_INFO("16 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
/*  //this->actual_goal=this->planner_.get_robot_goal();
  //this->doGoal(this->actual_goal);  
  if(debug_real_test_companion_){
    ROS_INFO("AkpLocalPlanner::computeVelocityCommands: this->we_have_companion_person_=%d=",this->we_have_companion_person_);
  }
  //planner iteration

 
  //Cperson_abstract::companion_reactive reactive_;
  // bool robot_plan_succed =  this->planner_.robot_plan(best_next_pose);
  
  if(this->we_have_companion_person_){
    robot_plan_succed =  this->planner_.robot_plan_companion2(best_next_pose,reactive_);
    this->move_base=true;

    if(debug_real_test_companion_robot_){
      ROS_INFO( "2(if) true this->we_have_companion_person_[in_robot_plan]  (this->move_base=true)! calc robot_plan_companion2 best_next_pose.v=%f",best_next_pose.v);
      ROS_INFO( "2 [in_robot_plan] (ROS) best_next_pose.w=%f",best_next_pose.w);
    }
    
  }else{

    this->robot_pose_.v=0.000000000001;
    this->robot_pose_.w=0;
    robot_plan_succed=true;
    this->move_base=false;
    best_next_pose=this->robot_pose_;
   
  // ROS_INFO( " this->robot_pose_.v=  %f, this->robot_pose_.w=%f ",  this->robot_pose_.v,this->robot_pose_.w );
  //robot_plan_succed=false;
  }

*/
  if(debug_real_test_companion_){
    ROS_INFO("AkpLocalPlanner::computeVelocityCommands: reactive_=%d=",reactive_);
  }
  //save_best_next_pose_= // solo para ver el marker de a donde intenta ir.
//ROS_INFO("17 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
  if(debug_antes_subgoals_entre_AKP_goals_){
    ROS_INFO( "best_next_pose.x=  %f ", best_next_pose.x );
    ROS_INFO( "best_next_pose.y=  %f ", best_next_pose.y ); 
  }

  if(debug_real_test_companion_){
    ROS_INFO( "robot plan  %s in time %f", robot_plan_succed ? "true" : "false", ros::Time::now().toSec()-t );
  }
//ROS_INFO("18 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
  best_next_pose_companion_markers_=best_next_pose; // for companion! (luego para cambiar el goal del robot, tendrá que ser aquí, donde cambie su posicion con el angulo entre robot y persona)

  //ROS_INFO( "robot plan  %s at velocity %f", robot_plan_succed ? "true" : "false",  best_next_pose.v );
 //ROS_INFO("19 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
  //Heuristic to average velocities and detect local minima problems
  if(this->move_base)
    velocities_.push_back( robot_pose_.v );

//ROS_INFO("20 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
  //pop front old velocities; in setPlan, a reset of the vector is done: 5 samples ~ 1seg
  if( velocities_.size() > 5 )
    velocities_.pop_front();

  // calculate average
  double avg_v(0.0);
  for( unsigned int i = 0; i < velocities_.size(); ++i )
    avg_v += velocities_[i];

//ROS_INFO("21 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");

  //detects abnormal behavior, that is, stopping robot
  //ROS_INFO("avergae velocity = %f of size %d", avg_v/5.0, (int)velocities_.size());
  if( fabs(avg_v) < 5.0*0.1 && velocities_.size() > 4 )
  {
    //robot_plan_succed = false;
    ROS_INFO( "OJOOOOOOOOOOOOOOOOOOOOOOOOO!!! robot plan would be invalidated"); // es de Gonzalo, no se que de minimos para saltarselos, pero a mi no me va bien con el robot!
  }

  /*if((sqrt(best_next_pose.v*best_next_pose.v)<0.07)&&(sqrt(best_next_pose.w*best_next_pose.w)<0.07)){
	ROS_INFO("IN mobe_base_false por_vel_peque! 22 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");

	this->robot_pose_.v=0.000000000001;
    	this->robot_pose_.w=0;
    	robot_plan_succed=true;
    	this->move_base=false;
    	best_next_pose=this->robot_pose_;
  }*/
	
//ROS_INFO("22 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");

  if(debug_goal_person_){
   ROS_INFO( " !!!          DISTANCE=%f",best_person_dest_spoin.distance(robot_position_act));
  }

  if((this->move_base)&&(best_person_dest_spoin.distance(robot_position_act)>distance_to_aproach_person_goal_))// distance_to_aproach_person_goal_=0.5; 
  {
    cmd_vel.linear.x  = best_next_pose.v;
    cmd_vel.angular.z = best_next_pose.w;

    if(debug_real_test_companion_robot_){
      if(!robot_plan_succed){
        ROS_INFO( "4 if(!robot_plan_succed) move base true if (this->move_base)!robot_plan_succed (ROS) best_next_pose.v=%f",best_next_pose.v);
        ROS_INFO( "4 if(!robot_plan_succed)if (ROS) best_next_pose.w=%f",best_next_pose.w);
      }else{
        ROS_INFO( "4.2 else (this->move_base) else (ROS) best_next_pose.v=%f",best_next_pose.v);
        ROS_INFO( "4.2 (ROS) best_next_pose.w=%f",best_next_pose.w);
      }
    }

  }
  else
  {
    cmd_vel.linear.x  = 0.0;
    cmd_vel.angular.z = 0.0;

    if(debug_real_test_companion_robot_){
        if(!robot_plan_succed){
          ROS_INFO( "5 if(!robot_plan_succed) move_base false (else this->move_base) !robot_plan_succed (ROS) best_next_pose.v=%f",best_next_pose.v);
          ROS_INFO( "5 (ROS) best_next_pose.w=%f",best_next_pose.w);
        }else{
          ROS_INFO( "5.2 else (else this->move_base) else (ROS) best_next_pose.v=%f",best_next_pose.v);
          ROS_INFO( "5.2 (ROS) best_next_pose.w=%f",best_next_pose.w);
        }
    }
  }
  publishPlan(this->global_plan_, g_plan_pub_);
  publishPlan(this->local_plan_, l_plan_pub_);

//ROS_INFO("23 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");



 /* if(){ // go to person moving goal. If reach goal, stay on the goal.
    cmd_vel.linear.x  = 0.0;
    cmd_vel.angular.z = 0.0;

   }*/

  //t = ros::Time::now().toSec();//just to measure the drawing of markers
/* 
 //fill planning markers
  if( vis_mode_ > 0 && !frozen_mode_  )
  {
    MarkerArray_msg_.markers.clear();
    MarkerArray_msg_comp_markers_.markers.clear();
    MarkerArray_msg_m2_.markers.clear();
    MarkerArray_msg_people_prediction_time_.markers.clear();

    //minimal mode: scene and best path
    //ROS_INFO( "entering fill best path 2d" );
    // if(reactive_==Cperson_abstract::Akp_planning){// unico caso en que hay planning y se pueden generar los markers del best path.

    Sdestination goal_out_person=planner_.get_person_companion_goal_out();

    best_next_pose_robot_companion_markers_.pose.position.x = best_next_pose_companion_markers_.x;
    best_next_pose_robot_companion_markers_.pose.position.y = best_next_pose_companion_markers_.y;
    //best_next_pose_robot_companion_markers_.pose.position.x = goal_out_person.x;
    //best_next_pose_robot_companion_markers_.pose.position.y = goal_out_person.y;
    best_next_pose_robot_companion_markers_.id = 30;
    //MarkerArray_msg_.markers.push_back( best_next_pose_robot_companion_markers_ ); //esfera que marca el siguiente goal del robot, hacia donde va!
    MarkerArray_msg_comp_markers_.markers.push_back( best_next_pose_robot_companion_markers_ );

    fill_best_path_2d();
    // }
    //ROS_INFO( "entering scene markers" );
    fill_scene_markers();
    if( vis_mode_ > 1 )//normal mode 2d features
    {
      fill_forces_markers();
      fill_laser_obstacles();
      
      //if(reactive_==Cperson_abstract::Akp_planning){// unico caso en que hay planning y se pueden generar los markers del best path.
        fill_planning_markers_2d(); // seguramente que tambien da problemas...
        fill_people_prediction_markers_3d(); // provisional. TODO. QUITAr
      //}    

    }
    if ( vis_mode_ > 2 )//super mode 3d features
    {
      //fill_people_prediction_markers_2d();
      fill_people_prediction_markers_3d();
      //if(reactive_==Cperson_abstract::Akp_planning){// unico caso en que hay planning y se pueden generar los markers del best path.
        fill_planning_markers_3d(); // mirar...
      //}
    }
    if ( vis_mode_ > 3 )//super mode, plooting potentials and so on
    {
      //TODO
    }
    fill_robot_companion_markers(); // de momento inhabilitado, pq ya se ve desde robot y persona simulada directamente.

    fill_people_prediction_markers_2d_companion();
    // fill_people_prediction_markers_3d_companion();
    //if(reactive_==Cperson_abstract::Akp_planning){// unico caso en que hay planning y se pueden generar los markers del best path.
    //fill_planning_markers_3d_companion();

  } 
  fill_test_markers(); // made by ely

  this->markers_publisher_.publish(this->MarkerArray_msg_);
  this->markers_publisher_comp_markers_.publish(this->MarkerArray_msg_comp_markers_);
  this->markers_publisher_m2_.publish(this->MarkerArray_msg_m2_);
  this->markers_publisher_people_prediction_time_.publish(this->MarkerArray_msg_people_prediction_time_);
*/

  // publish robot costs TODO to be deprecated
  //double work_robot, work_persons;
  //this->planner_.get_navigation_instant_work( work_robot, work_persons );
  //Float64_msg_.data[0] = work_robot;
  //ROS_DEBUG("FoceLocalPlanner::computeVelocityCommands: cmd_vel=(%f,%f)", cmd_vel.linear.x,  cmd_vel.angular.z);
  
  // publish navigation costs dist[0], orient[1], robot[2], ppl[3], obst[4]
  //ROS_INFO( "entering costs" );
  std::vector<double> costs;  // TODO: en teoria el companion_cost. esta aquí add de dentro, pero no segura...
  scene_.get_akp_for_person_companion()->get_navigation_cost_values( costs);
  Float64_msg_.data = costs;
  scene_.get_akp_for_person_companion()->get_navigation_mean_cost_values( costs);
  Float64_msg_.data.insert( Float64_msg_.data.end(), costs.begin(), costs.end() );
  scene_.get_akp_for_person_companion()->get_navigation_std_cost_values( costs);
  Float64_msg_.data.insert( Float64_msg_.data.end(), costs.begin(), costs.end() );
  this->cost_params_publisher_.publish(this->Float64_msg_);
  //ROS_INFO( "current markers = %f", ros::Time::now().toSec()-t);
  this->planner_mutex_exit();
  //ROS_INFO("AkpLocalPlanner::ComputeVelocites: Exit"); 
//ROS_INFO("24 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
  if(debug_real_test_companion_){
    ROS_INFO( "time of processing (todo planing==computeVelocityCommands)= %f", (ros::Time::now()-now).toSec());
    std::cout <<std::endl<<std::endl;
  }
//ROS_INFO("25 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
		this->doGoal(); // mirar si aquí o más arriba...
//ROS_INFO("26 [Person_companion DABO] ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
  return robot_plan_succed;
}

bool AkpLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if(debug_goal_person_){
     ROS_INFO("[DABO] !!!!!!!!!!!!!!!!!!! ENTRO EN AkpLocalPlanner::setPlan!!!");
  }
  //ROS_INFO("!!!!!!!!!!!!!!!!!!! ENTRO EN AkpLocalPlanner::setPlan!!!");
  // (1) funcion, local planer. Da las posiciones del local plan, path, a seguir por el robot. la que toca de verdad la libreria de Gonzalo. El plan que el calcula.
  // ROS_INFO("AkpLocalPlanner::setPlan");
  if(!initialized_)
  {
    ROS_ERROR("AkpLocalPlanner::setPlan: This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }
  velocities_.clear();//clears the average of velocities when new plans are calculated
  this->global_plan_.clear();
  // robot_goal=this->planner_.get_robot_goal(); // (ely) puesto para petarme el goal externo del rviz!!!
  //orig_global_plan.pose.position.x=robot_goal.x;
  // orig_global_plan.pose.position.y=robot_goal.y;
  this->global_plan_ =  orig_global_plan;
  if(debug_person_companion_){
  	ROS_INFO("AkpLocalPlanner::setPlan: global_plan_ points.x= %f ; points.y= %f", orig_global_plan.back().pose.position.x, orig_global_plan.back().pose.position.y);
  	ROS_INFO("AkpLocalPlanner::setPlan: global_plan_ points %d", uint(this->global_plan_.size()));
  }
  // ROS_INFO("AkpLocalPlanner::setPlan: global_plan_ points %d", uint(this->global_plan_.size()));
  // slice global plan into subgoals
  if ( goal_providing_mode_ == AkpLocalPlanner::Slicing_global )
    slice_plan( );

  //else , there is no need to do anything
  return true;
}

void AkpLocalPlanner::slice_plan()
{
  if(debug_goal_person_){
  ROS_INFO("ENTRO EN AkpLocalPlanner::slice_plan()");
  }
  //ROS_INFO("ENTRO EN AkpLocalPlanner::slice_plan!!!");

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
			//ROS_INFO("entro en if!!!" );
      //check if it near the previous on and then discard
      dx = sliced_global_plan_.back().pose.position.x - global_plan_[i].pose.position.x;
      dy = sliced_global_plan_.back().pose.position.y - global_plan_[i].pose.position.y;
      accumulated_orientation = 0.0;
      Spoint robot_act_global_goal_=Spoint(global_plan_[i].pose.position.x, global_plan_[i].pose.position.y);
      if ( ( dx*dx + dy*dy > 1.0 ) && (this->robot_pose_.distance( robot_act_global_goal_ ) > planner_.get_distance_to_stop()) )
      {
        sliced_global_plan_.push_back( global_plan_[i] );
        //ROS_INFO( "Subgoal in (%f, %f) at i = %d", sliced_global_plan_.back().pose.position.x, sliced_global_plan_.back().pose.position.y, i );
      }
    }  
  }
  
  //for( unsigned int i=0; i < sliced_global_plan_.size(); ++i )
  //  ROS_INFO( "AkpLocalPlanner::slice_plan: Subgoal %d in (%f, %f)", i, sliced_global_plan_[i].pose.position.x, sliced_global_plan_[i].pose.position.y );
 //ROS_INFO("sliced_global_plan_.size()= %d " , sliced_global_plan_.size());

      
}

bool AkpLocalPlanner::isGoalReached()
{ 
  if(debug_antes_subgoals_entre_AKP_goals_){
   ROS_INFO(" [DABO] ENTRO EN AkpLocalPlanner::isGoalReached!!!");
  }
  // (3) funcion, local planer. Comprueba si el robot llega al goal. Si no llega se vuelve a llamar a la funcion de las velocidades, para conseguir que el robot llegue al goal.
  // ROS_INFO("AkpLocalPlanner::isGoalReached");
  bool ok=false;
  //bool ok=true;
  double goal_x=0.0;
  double goal_y=0.0;

  if(!initialized_){
    ROS_ERROR("AkpLocalPlanner::isGoalReached: This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }
  if(debug_antes_subgoals_entre_AKP_goals_){
    ROS_INFO("AkpLocalPlanner::setPlan: global_plan_ points.x= %f ; points.y= %f", this->global_plan_.back().pose.position.x, this->global_plan_.back().pose.position.y);
  }
  //TODO switch to different subgoal generation: 1) croping at local window. 2) calculating the set of subgoals
   if( goal_providing_mode_ == AkpLocalPlanner::Slicing_global )
  {

    //ROS_INFO("ENTRO EN AkpLocalPlanner::isGoalReached!!! (caso if)");
    //robot_goal=this->planner_.get_robot_goal();
    //propose the new goal as the last of
    this->robot_goal_.x = sliced_global_plan_.back().pose.position.x;
    this->robot_goal_.y = sliced_global_plan_.back().pose.position.y;  
    //ROS_INFO("(out if) robot_goal_.x=%f; robot_goal_.y=%f",robot_goal_.x,robot_goal_.y);
		//ROS_INFO("this->robot_pose_.distance( this->robot_goal_ )=%f",this->robot_pose_.distance( this->robot_goal_ ));
		//ROS_INFO("planner_.get_distance_to_stop()=%f",planner_.get_distance_to_stop());
		//ROS_INFO("sliced_global_plan_.size()=%d",sliced_global_plan_.size());
    //identify of it is the last subgoal is reached and then propose a new one if any
   // if( this->robot_pose_.distance( this->robot_goal_ ) < planner_.get_distance_to_stop() && sliced_global_plan_.size() > 1  )
if( this->robot_pose_.distance( this->robot_goal_ ) > planner_.get_distance_to_stop() && sliced_global_plan_.size() > 1  )
    {
			
      	sliced_global_plan_.pop_back();
      	this->robot_goal_.x = sliced_global_plan_.back().pose.position.x;
      	this->robot_goal_.y = sliced_global_plan_.back().pose.position.y;  
				//ROS_INFO("(in if) robot_goal_.x=%f; robot_goal_.y=%f",robot_goal_.x,robot_goal_.y);

    }
  }
  else // if( goal_providing_mode_ == AkpLocalPlanner::Crop_local_window)
  {
  
    //ROS_INFO("ENTRO EN AkpLocalPlanner::isGoalReached!!! (caso else)");
    //transform plan from global to local coordinates, and crop it to the local costmap window
  	//this->robot_goal_.x=this->global_plan_.back().pose.position.x;
 		//this->robot_goal_.y=this->global_plan_.back().pose.position.y;
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

/*
// Caso anterior gonzalo y ahora para hacer que la person companion navegue usando el akp del robot.
  this->planner_mutex_enter();
  this->planner_.set_robot_goal(this->robot_goal_); // es el goal externo!, pero queremos trasladarlo a los goals automaticos de la persona!

  if(this->robot_pose_.distance( this->robot_goal_ ) < this->xy_goal_tolerance
     && this->robot_pose_.v < this->v_goal_tolerance)
  {
    ok=true;
    ROS_INFO("AkpLocalPlanner::isGoalReached: GOAL REACHED x,y: %f, %f at distance %f", robot_goal_.x, robot_goal_.y, this->robot_pose_.distance( this->robot_goal_ ));
  }

  this->planner_mutex_exit();
*/


    // caso akp_planner_companion. => es mi caso, pero SIEMPRE entrando!



  this->planner_mutex_enter();
  //if((reactive_==Cperson_abstract::Akp_planning)&&external_goal_){
  //if(reactive_==Cperson_abstract::Akp_planning){
   if(debug_antes_subgoals_entre_AKP_goals_){
      ROS_INFO("AkpLocalPlanner::isGoalReached: ENTRO EN set_robot_external_goal ROS!!! this->robot_goal_.x=%f ; this->robot_goal_.y= %f",this->robot_goal_.x,this->robot_goal_.y);
      ROS_INFO("AkpLocalPlanner::isGoalReached: ENTRO EN set_robot_external_goal ROS!!! reactive_=%d=",reactive_);
  }
   // this->planner_.set_robot_external_goal(this->robot_goal_); // para darle el goal externo desde el rviz!!!!
  //  this->planner_.set_robot_external_goal_fix(this->robot_goal_); // para darle el goal externo desde el rviz!!!!
 	//this->scene_.get_akp_for_person_companion()->set_robot_external_goal(this->robot_goal_); // para darle el goal externo desde el rviz!!!!
  // this->scene_.get_akp_for_person_companion()->set_robot_external_goal_fix(this->robot_goal_); // para darle el goal externo desde el rviz!!!!
 //ROS_INFO(" [NODO extern goal] AkpLocalPlanner::isGoalReached: ENTRO EN set_robot_external_goal ROS!!! this->robot_goal_.x=%f ; this->robot_goal_.y= %f",this->robot_goal_.x,this->robot_goal_.y);
	this->scene_.get_akp_for_person_companion()->set_robot_goal(this->robot_goal_);
	this->scene_.get_akp_for_person_companion()->set_robot_goal_person_goal_global_plan(this->robot_goal_);
 // }

  if(this->robot_pose_.distance( this->robot_goal_ ) < this->xy_goal_tolerance
     && this->robot_pose_.v < this->v_goal_tolerance)
  {
    //ok=true;
   // ROS_INFO("AkpLocalPlanner::isGoalReached: GOAL REACHED x,y: %f, %f at distance %f", robot_goal_.x, robot_goal_.y, this->robot_pose_.distance( this->robot_goal_ ));
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
 // ROS_INFO(" [DABO] AkpLocalPlanner::scan_callback: New Message Received"); 

  this->rscan_mutex_enter(); 
  this->rscan = *msg;
  this->rscan_received=true;
  this->rscan_mutex_exit(); 
}

std::vector<Spoint> AkpLocalPlanner::scan2points(const sensor_msgs::LaserScan scan)
{
  
  header_point_.header= scan.header;  // variable for companion (ely)

  std::vector<Spoint> points;
  std::string source_frame = scan.header.frame_id;
  std::string target_frame = this->fixed_frame;
  ros::Time target_time    = scan.header.stamp;

  //double actual_workspace_radi=this->planner_.get_workspace_radii();
  // ROS_INFO("AkpLocalPlanner::scan2points: actual_workspace_radi=%f", actual_workspace_radi);

  // double laser_obstacle_marge_distance=actual_workspace_radi+1; // antes con gonzalo este margen era= 15.0;
  //ROS_INFO("AkpLocalPlanner::scan2points: laser_obstacle_marge_distance=%f", laser_obstacle_marge_distance);


  //  ROS_INFO("AkpLocalPlanner::scan2points: scan.ranges.size()=%d", scan.ranges.size());

  if(source_frame!=target_frame){
    //ROS_INFO("AkpLocalPlanner::scan2points:transform: %s-->%s", source_frame.c_str(), target_frame.c_str());
    try
    {
    //ROS_INFO("AkpLocalPlanner::scan2points: 2 target_time=%f",target_time.toSec());
      bool tf_exists = this->tf_->waitForTransform(target_frame, source_frame, target_time, ros::Duration(1), ros::Duration(0.01));
      if(tf_exists)
      {

        //ROS_INFO("tf exist");

        geometry_msgs::PointStamped pointIn;
        geometry_msgs::PointStamped pointOut;
        pointIn.header = scan.header;
        double t;
        for( unsigned int i = 0; i < scan.ranges.size(); ++i)
        {
         // ROS_INFO("i=%d",i);
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
  }else{
     double t;
     for( unsigned int i = 0; i < scan.ranges.size(); ++i)
     {
          if ( scan.ranges[i]  < 15.0  && scan.ranges[i] > 0.05)
          {
            t = scan.angle_min + (double)i*scan.angle_increment;   
            points.push_back( Spoint( scan.ranges[i] * cos( t ), scan.ranges[i] * sin( t ) ) );
          }
     }


  }

 //ROS_INFO("AkpLocalPlanner::scan2points: points.size()=%d", points.size());

  return points;
}

void AkpLocalPlanner::odom_callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
  //ROS_INFO("[DABO] AkpLocalPlanner::odom_callback: New Message Received");

  this->odom_mutex_enter();
  this->robot_pose_.v = msg->twist.twist.linear.x;
  this->robot_pose_.w = msg->twist.twist.angular.z;

  //Get Robot position: transform empty/zero pose from base_link to map
  std::string target_frame = this->fixed_frame;
  this->robot_frame  = "/" +this->robot_companion_person_name_ + "/base_link"; 
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
    //ROS_INFO("AkpLocalPlanner::odom_callback: YES transform: %s-->%s", source_frame.c_str(), target_frame.c_str());
     // ROS_INFO("AkpLocalPlanner::odom_callback: robot pose x,y,th,v,w: %f, %f, %f, %f, %f", this->robot_pose_.x, this->robot_pose_.y, this->robot_pose_.theta, this->robot_pose_.v, this->robot_pose_.w);
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

//  this->odom_subscriber_other_person_comp_of_group_   = this->public_node_handle_.subscribe<nav_msgs::Odometry>("odom_other_person_of_group"  , 1, boost::bind(&AkpLocalPlanner::odom_callback_other_person_comp_of_group, this, _1));


void AkpLocalPlanner::odom_callback_other_person_comp_of_group(const nav_msgs::Odometry::ConstPtr& msg) 
{

  if(group_simulation_now_){
   // ROS_INFO("[DABO] AkpLocalPlanner::odom_callback_other_person_comp_of_group: New Message Received");

    this->odom_other_person_comp_of_group_mutex_enter();
    this->robot_pose_other_comp_pers_.v = msg->twist.twist.linear.x;
    this->robot_pose_other_comp_pers_.w = msg->twist.twist.angular.z;

    //Get Robot position: transform empty/zero pose from base_link to map
    std::string target_frame = this->fixed_frame;
    this->robot_frame  = "/" +this->robot_companion_person_name_ + "/base_link"; // Necesito frame de la otra persona que acompaña a tibi
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
        this->robot_pose_other_comp_pers_.x = poseOut.pose.position.x;
        this->robot_pose_other_comp_pers_.y = poseOut.pose.position.y;
        this->robot_pose_other_comp_pers_.time_stamp = poseOut.header.stamp.toSec();

        //vector of the orientation
        poseIn.pose.position.x = 1.0;
        this->tf_->transformPose(target_frame, poseIn, poseOut);
        this->robot_pose_other_comp_pers_.theta = atan2(poseOut.pose.position.y - this->robot_pose_other_comp_pers_.y , poseOut.pose.position.x - this->robot_pose_other_comp_pers_.x);
        //ROS_INFO("AkpLocalPlanner::robot_pose_other_comp_pers_.theta: %f",this->robot_pose_other_comp_pers_.theta);
        //this->planner_.update_robot(this->robot_pose_);//updated in main
      //ROS_INFO("AkpLocalPlanner::odom_callback: YES transform: %s-->%s", source_frame.c_str(), target_frame.c_str());
       // ROS_INFO("AkpLocalPlanner::odom_callback: robot pose x,y,th,v,w: %f, %f, %f, %f, %f", this->robot_pose_.x, this->robot_pose_.y, this->robot_pose_.theta, this->robot_pose_.v, this->robot_pose_.w);
      }
      else
      {
        ROS_WARN("AkpLocalPlanner::odom_callback_other_person_comp_of_group: No transform: %s-->%s", source_frame.c_str(), target_frame.c_str());
      }
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("AkpLocalPlanner::odom_callback_other_person_comp_of_group: %s",ex.what());
    }

    this->odom_other_person_comp_of_group_mutex_exit();
    //ROS_INFO("AkpLocalPlanner::odom_callback_other_person_comp_of_group: Exit");

  }
}




void AkpLocalPlanner::odom_callback_tibi_robot(const nav_msgs::Odometry::ConstPtr& msg) 
{
  //ROS_INFO("[TIBI en dabo] AkpLocalPlanner::odom_TIBI_en_dabo_callback: New Message Received");

  this->odom_tibi_mutex_enter();
  this->robot_pose_tibi_.v = msg->twist.twist.linear.x;
  this->robot_pose_tibi_.w = msg->twist.twist.angular.z;

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
      this->robot_pose_tibi_.x = poseOut.pose.position.x;
      this->robot_pose_tibi_.y = poseOut.pose.position.y;
      this->robot_pose_tibi_.time_stamp = poseOut.header.stamp.toSec();

      //vector of the orientation
      poseIn.pose.position.x = 1.0;
      this->tf_->transformPose(target_frame, poseIn, poseOut);
      this->robot_pose_tibi_.theta = atan2(poseOut.pose.position.y - this->robot_pose_tibi_.y , poseOut.pose.position.x - robot_pose_tibi_.x);
      //this->planner_.update_robot(this->robot_pose_tibi_);//updated in main
    //ROS_INFO("AkpLocalPlanner::odom_callback: YES transform: %s-->%s", source_frame.c_str(), target_frame.c_str());
     // ROS_INFO("AkpLocalPlanner::odom_callback: robot pose x,y,th,v,w: %f, %f, %f, %f, %f", this->robot_pose_.x, this->robot_pose_.y, this->robot_pose_.theta, this->robot_pose_.v, this->robot_pose_.w);


//ROS_INFO("[TIBI en dabo] this->poseIn.x=%f; this->poseIn.y=%f;",poseIn.pose.position.x,poseIn.pose.position.y);
    }
    else
    {
      ROS_WARN("AkpLocalPlanner::odom_TIBI_en_dabo_callback: No transform: %s-->%s", source_frame.c_str(), target_frame.c_str());
    }
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("AkpLocalPlanner::odom_TIBI_en_dabo_callback: %s",ex.what());
  }

  this->odom_tibi_mutex_exit();
  //ROS_INFO("AkpLocalPlanner::odom_TIBI_en_dabo_callback: Exit");

  //ROS_INFO("[TIBI en dabo] this->robot_pose_tibi_.x=%f; this->robot_pose_tibi_.y=%f; v=%f; w=%f; time=%f; theta=%f",this->robot_pose_tibi_.x,this->robot_pose_tibi_.y,this->robot_pose_tibi_.v,this->robot_pose_tibi_.w,this->robot_pose_tibi_.theta,this->robot_pose_tibi_.time_stamp);


// this->planner_.update_robot(this->robot_pose_tibi_);
}




/*
void AkpLocalPlanner::tracks_callback(const iri_perception_msgs::detectionArray::ConstPtr& msg) 
{
  //ROS_INFO("[DABO] AkpLocalPlanner::tracks_callback: New Message Received"); 
  
  //ROS_INFO("AkpLocalPlanner::tracks_callback: msg->header.frame_id= %s",msg->header.frame_id.c_str());
  //ROS_INFO("AkpLocalPlanner::tracks_callback: this->fixed_frame= %s",this->fixed_frame.c_str());

  if(msg->header.frame_id == this->fixed_frame)
  {
    this->tracks_mutex_enter();
    //std::vector<SdetectionObservation> obs;
    this->obs.clear();
    std::vector<double> cov;
    cov.resize(16,0.0);
    //ROS_INFO("AkpLocalPlanner::tracks_callback: msg->detection.size() = %d",msg->detection.size());
    for( unsigned int i = 0; i< msg->detection.size(); ++i)
    {
      //ROS_INFO("AkpLocalPlanner::tracks_callback: id = %d, (x,y) = (%f, %f), (vx,vy) = (%f, %f), prob=%f", msg->detection[i].id, msg->detection[i].position.x, msg->detection[i].position.y, msg->detection[i].velocity.x, msg->detection[i].velocity.y, msg->detection[i].probability);
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
*/


void AkpLocalPlanner::params_values_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  ROS_INFO("AkpLocalPlanner::params_values_callback: new message");

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

void AkpLocalPlanner::reconfigureCallback(iri_simulated_person_companion_akp_local_planner::AkpLocalPlannerConfig &config, uint32_t level)
{
  ROS_INFO(" INI AkpLocalPlanner::reconfigureCallback");
  std::vector<double> params(5,0.0);
  if(setup_ && config.restore_defaults)
  {
    config = default_config_;
    //Avoid looping
    config.restore_defaults = false;
  }
  if(!setup_)
  {

    this->robot_goal_force_marker       = config.robot_goal_force_marker;
    this->robot_person_forces_marker    = config.robot_person_forces_marker;
    this->robot_obstacles_forces_marker = config.robot_obstacles_forces_marker;
    this->robot_resultant_force_marker  = config.robot_resultant_force_marker;

    default_config_ = config;
    setup_ = true;
    // change (ely) para puentear lo de move_base a true! (que me deje dar goals desde el principio!)
    
    this->planner_mutex_enter();
    this->move_base = config.move_base;
    //this->move_base = true;//config.move_base;
    if(debug_antes_subgoals_entre_AKP_goals_){
      std::cout << " AkpLocalPlanner::reconfigureCallback this->move_base ="<< this->move_base  << std::endl;
    }
    this->plan_mode_ = (Cplan_local_nav::plan_mode) config.plan_mode;
    this->scene_.get_akp_for_person_companion()->set_planning_mode(plan_mode_);
    this->scene_.get_akp_for_person_companion()->set_distance_mode((Cplan_local_nav::distance_mode)config.distance_mode );
    this->scene_.get_akp_for_person_companion()->set_global_mode((Cplan_local_nav::global_mode)config.global_mode );
    this->scene_.get_akp_for_person_companion()->set_number_of_vertex(config.number_vertex);
    // ROS_INFO("\n\n\n numer of vertex = %d\n\n\n\n" , config.number_vertex);

    this->scene_.get_akp_for_person_companion()->set_horizon_time( 7.0);
    this->planner_.set_horizon_time( 7.0 );

    this->scene_.get_akp_for_person_companion()->set_horizon_time( config.horizon_time );
    this->xy_goal_tolerance = config.xy_goal_tolerance;
    this->scene_.get_akp_for_person_companion()->set_xy_2_goal_tolerance( this->xy_goal_tolerance * this->xy_goal_tolerance);
    this->scene_.get_akp_for_person_companion()->set_distance_to_stop( config.distance_to_stop );
    this->v_goal_tolerance = config.v_goal_tolerance;
    this->scene_.get_akp_for_person_companion()->set_v_goal_tolerance( this->v_goal_tolerance );
    this->scene_.get_akp_for_person_companion()->set_plan_cost_parameters(config.cost_distance,
                              config.cost_orientation,
                              config.cost_w_robot,
                              config.cost_w_people,
                              config.cost_time,
                              config.cost_obs,
                              config.cost_old_path,
                              config.cost_l_minima);
    this->scene_.get_akp_for_person_companion()->set_robot_params(config.v_max, config.w_max, config.av_max, config.av_break, config.aw_max, config.platform_radii);
    this->vis_mode_ = config.vis_mode;
    this->frozen_mode_ = config.frozen_mode;
    //config ESFM paramters
    params[0] = config.esfm_k;//k
    params[1] = config.esfm_to_person_lambda;//lambda
    params[2] = config.esfm_to_person_A;//A
    params[3] = config.esfm_to_person_B;//B
    params[4] = config.esfm_d;//d
    this->scene_.get_akp_for_person_companion()->set_sfm_to_person( params );
    params[1] = config.esfm_to_robot_lambda;//lambda
    params[2] = config.esfm_to_robot_A;//A
    params[3] = config.esfm_to_robot_B;//B
    this->scene_.get_akp_for_person_companion()->set_sfm_to_robot( params );
    params[1] = config.esfm_to_obstacle_lambda;//lambda
    params[2] = config.esfm_to_obstacle_A;//A
    params[3] = config.esfm_to_obstacle_B;//B
    this->scene_.get_akp_for_person_companion()->set_sfm_to_obstacle( params );
    this->scene_.get_akp_for_person_companion()->set_min_v_to_predict( config.min_v_to_predict );//Cscene_bhmip
    this->scene_.get_akp_for_person_companion()->set_ppl_collision_mode( config.ppl_collision_mode );
    this->scene_.get_akp_for_person_companion()->set_pr_force_mode( config.pr_force_mode );
    goal_providing_mode_ = (AkpLocalPlanner::Goal_providing_mode)config.goal_providing_mode;
    slicing_path_diff_orientation_ = config.slicing_diff_orientation;

    // ini companion config variables.

    this->scene_.get_akp_for_person_companion()->set_id_person_companion(config.id_person_companion);
    this->scene_.get_akp_for_person_companion()->set_robot_person_proximity_distance(config.proximity_distance_between_robot_and_person);
    this->scene_.get_akp_for_person_companion()->set_proximity_distance_tolerance(config.proximity_distance_tolerance);
    this->scene_.get_akp_for_person_companion()->set_additional_distance_companion_sphere(config.add_dist_companion_sphere);
    this->scene_.get_akp_for_person_companion()->set_proximity_goals_robot_and_person(config.proximity_goals_robot_and_person_x,config.proximity_goals_robot_and_person_y);
    this->scene_.get_akp_for_person_companion()->set_offset_attractive(config.offset_attractive_state);
    this->scene_.get_akp_for_person_companion()->set_force_obs_max(config.force_obs_max_x,config.force_obs_max_y);
    // this->scene_.get_akp_for_person_companion()->set_real_companion_angle(config.real_companion_angle);
    this->scene_.get_akp_for_person_companion()->set_person_goal_percentage(config.person_goal_percentage);
    this->scene_.get_akp_for_person_companion()->set_id_person_companion_Cprediction_bhmip(config.id_person_companion);
    this->scene_.get_akp_for_person_companion()->set_overpas_obstacles_behind_person(config.overpas_obstacles_behind_person);
    this->scene_.get_akp_for_person_companion()->set_anisotropy_threshold(config.anisotropy_threshold);
    this->scene_.get_akp_for_person_companion()->set_max_d_to_detect_laser_obs(config.detection_laser_obstacle_distances);    
    external_goal_=config.external_goal;
    if(config.external_goal==true){
       this->scene_.get_akp_for_person_companion()->set_companion_same_person_goal(false);
    }else{
      this->scene_.get_akp_for_person_companion()->set_companion_same_person_goal(true);
    }
    this->scene_.get_akp_for_person_companion()->set_max_asos_point_to_person(config.in_max_asos_point_to_person);

    this->scene_.get_akp_for_person_companion()->set_save_results_in_file(config.save_results_in_file);
    this->scene_.get_akp_for_person_companion()->set_meters_to_goal_to_save_results_in_file(config.metros_al_goal);
    this->scene_.get_akp_for_person_companion()->set_mode_velocity(config.mode_velocity);
 /* this->planner_.set_results_filename(config.results_filename);
    this->planner_.set_reduce_max_vel_dist(config.reduce_max_vel_sim);
    this->planner_.set_marge_in_distance(config.marge_in_distance);
    this->planner_.set_marge_angle_companion(config.marge_angle_companion);
    debug_real_test_companion_robot_=config.debug_robot_ROS_Info;
    this->planner_.set_mode_debug_cout(config.debug_robot_cout_library);
    this->planner_.set_mode_debug_file(config.debug_robot_file);
    this->planner_.set_debug_filename(config.debug_filename);
    this->planner_.set_externa_force_k_near_goal_akp(config.set_external_near_k);
    this->planner_.set_externa_force_k_far_goal_akp(config.set_external_far_k);
    this->planner_.set_ex_max_dist_to_near_goal_force_akp(config.ex_max_dist_to_near_goal_force);
    this->planner_.set_mode_step_near(config.mode_step_near);
    this->planner_.set_out_index_step(config.out_index_step);
    this->planner_.only_comp_people_vel_and_robot_poses(config.debug_only_robot_point_comp_person_point);
    this->planner_.set_person_radi_amp(config.person_radi_amp);
    this->planner_.set_obstacle_radi_amp(config.obstacle_radi_amp);
    this->planner_.set_ini_vel_to_increment_angle(config.ini_vel_to_increment_angle);
   */ // end companion config variables.
    this->scene_.get_akp_for_person_companion()->set_evaluate_costs_filename(config.evaluate_costs_filename);

    // get person goal ID
    id_person_goal_=config.person_goal_id;
    this->scene_.set_id_person_goal(config.person_goal_id);
    this->scene_.get_akp_for_person_companion()->set_id_person_goal(config.person_goal_id);
    //this->scene_.get_akp_for_person_companion()->set_id_person_goal(config.person_goal_id);
	  //id_person_goal_=2;   
	this->robot_companion_person_name_=config.robot_sim_comp_pers_name;
    	this->robot_companion_person_name_out_node_=config.other_person_of_group_comp_sim_name;

	this->scene_.set_other_companion_person_id(config.id_person_other_person_of_group_companion);
	config.id_person_other_person_of_group_companion=config.id_person_other_person_of_group_companion;
	this->scene_.set_CsceneSim_group_go_to_interact_with_other_person(config.conf_bool_group_go_to_interact_with_other_person);
	  /*static tf::TransformBroadcaster br;
  	tf::Transform transform;
  	transform.setOrigin( tf::Vector3(robot_ini_pose_x_, robot_ini_pose_y_, 0.0) );
  	//ROS_INFO("(ROS) AkpLocalPlanner::fill_robot_companion_markers:  transform.pose.position.x=%f",transform.pose.position.x);
  	//ROS_INFO("(ROS) AkpLocalPlanner::fill_robot_companion_markers: transform.pose.position.y=%f", transform.pose.position.y);   
  	//ROS_INFO("(ROS) AkpLocalPlanner:: 1");     
  	transform.setRotation( tf::Quaternion(0, 0, robot_ini_pose_theta_) ); // msg->theta= buscar y guardar por arriba orientación robot.
	  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/"+this->robot_companion_person_name_+"/base_footprint"));*/
    this->planner_mutex_exit();
  }
  else if(setup_)
  {
    
    this->planner_mutex_enter();
    this->move_base = config.move_base;
    //this->move_base = true;//config.move_base;
    if(debug_antes_subgoals_entre_AKP_goals_){
      std::cout << " AkpLocalPlanner::reconfigureCallback this->move_base ="<< this->move_base  << std::endl;
    }
    this->plan_mode_ = (Cplan_local_nav::plan_mode) config.plan_mode;
    this->scene_.get_akp_for_person_companion()->set_planning_mode(plan_mode_);
    this->scene_.get_akp_for_person_companion()->set_distance_mode((Cplan_local_nav::distance_mode)config.distance_mode );
    this->scene_.get_akp_for_person_companion()->set_global_mode((Cplan_local_nav::global_mode)config.global_mode );
    this->scene_.get_akp_for_person_companion()->set_number_of_vertex(config.number_vertex);
    if(debug_antes_subgoals_entre_AKP_goals_){
      ROS_INFO("\n\n\n numer of vertex = %d\n\n\n\n" , config.number_vertex);
    }
    this->scene_.get_akp_for_person_companion()->set_horizon_time( 7.0);
    this->planner_.set_horizon_time( 7.0 );
    // this->planner_.set_horizon_time( config.horizon_time );
    if(debug_antes_subgoals_entre_AKP_goals_){
      ROS_INFO("\n\n\n horizon_time = %f\n\n\n\n" , config.horizon_time );
    }
    this->xy_goal_tolerance = config.xy_goal_tolerance;
    this->planner_.set_xy_2_goal_tolerance( this->xy_goal_tolerance * this->xy_goal_tolerance);
    this->planner_.set_distance_to_stop( config.distance_to_stop );
    this->v_goal_tolerance = config.v_goal_tolerance;
    this->scene_.get_akp_for_person_companion()->set_v_goal_tolerance( this->v_goal_tolerance );
    this->scene_.get_akp_for_person_companion()->set_plan_cost_parameters(config.cost_distance,
                              config.cost_orientation,
                              config.cost_w_robot,
                              config.cost_w_people,
                              config.cost_time,
                              config.cost_obs,
                              config.cost_old_path,
                              config.cost_l_minima);
    this->scene_.get_akp_for_person_companion()->set_robot_params(config.v_max, config.w_max, config.av_max, config.av_break, config.aw_max, config.platform_radii);
    this->vis_mode_ = config.vis_mode;
    this->frozen_mode_ = config.frozen_mode;
    //config ESFM paramters
    params[0] = config.esfm_k;//k
    params[1] = config.esfm_to_person_lambda;//lambda
    params[2] = config.esfm_to_person_A;//A
    params[3] = config.esfm_to_person_B;//B
    params[4] = config.esfm_d;//d
    this->scene_.get_akp_for_person_companion()->set_sfm_to_person( params );
    params[1] = config.esfm_to_robot_lambda;//lambda
    params[2] = config.esfm_to_robot_A;//A
    params[3] = config.esfm_to_robot_B;//B
    this->scene_.get_akp_for_person_companion()->set_sfm_to_robot( params );
    params[1] = config.esfm_to_obstacle_lambda;//lambda
    params[2] = config.esfm_to_obstacle_A;//A
    params[3] = config.esfm_to_obstacle_B;//B
    this->scene_.get_akp_for_person_companion()->set_sfm_to_obstacle( params );
    this->scene_.get_akp_for_person_companion()->set_min_v_to_predict( config.min_v_to_predict );//Cscene_bhmip
    this->scene_.get_akp_for_person_companion()->set_ppl_collision_mode( config.ppl_collision_mode );
    this->scene_.get_akp_for_person_companion()->set_pr_force_mode( config.pr_force_mode );
    goal_providing_mode_ = (AkpLocalPlanner::Goal_providing_mode)config.goal_providing_mode;
    slicing_path_diff_orientation_ = config.slicing_diff_orientation;

    // ini companion config variables.

    this->scene_.get_akp_for_person_companion()->set_id_person_companion(config.id_person_companion);
    this->scene_.get_akp_for_person_companion()->set_robot_person_proximity_distance(config.proximity_distance_between_robot_and_person);
    this->scene_.get_akp_for_person_companion()->set_proximity_distance_tolerance(config.proximity_distance_tolerance);
    this->scene_.get_akp_for_person_companion()->set_additional_distance_companion_sphere(config.add_dist_companion_sphere);
    this->scene_.get_akp_for_person_companion()->set_proximity_goals_robot_and_person(config.proximity_goals_robot_and_person_x,config.proximity_goals_robot_and_person_y);
    this->scene_.get_akp_for_person_companion()->set_offset_attractive(config.offset_attractive_state);
    this->scene_.get_akp_for_person_companion()->set_force_obs_max(config.force_obs_max_x,config.force_obs_max_y);
    // this->planner_.set_real_companion_angle(config.real_companion_angle);
    this->scene_.get_akp_for_person_companion()->set_person_goal_percentage(config.person_goal_percentage);
    this->scene_.get_akp_for_person_companion()->set_id_person_companion_Cprediction_bhmip(config.id_person_companion);
    this->scene_.get_akp_for_person_companion()->set_overpas_obstacles_behind_person(config.overpas_obstacles_behind_person);
    this->scene_.get_akp_for_person_companion()->set_anisotropy_threshold(config.anisotropy_threshold);
    this->scene_.get_akp_for_person_companion()->set_max_d_to_detect_laser_obs(config.detection_laser_obstacle_distances);
    external_goal_=config.external_goal;
    if(config.external_goal==true){
       this->scene_.get_akp_for_person_companion()->set_companion_same_person_goal(false);
    }else{
      this->scene_.get_akp_for_person_companion()->set_companion_same_person_goal(true);
    }
    this->scene_.get_akp_for_person_companion()->set_max_asos_point_to_person(config.in_max_asos_point_to_person);
    this->scene_.get_akp_for_person_companion()->set_save_results_in_file(config.save_results_in_file);
    this->scene_.get_akp_for_person_companion()->set_meters_to_goal_to_save_results_in_file(config.metros_al_goal);
    this->scene_.get_akp_for_person_companion()->set_mode_velocity(config.mode_velocity);
    /*   this->planner_.set_results_filename(config.results_filename);
    this->planner_.set_reduce_max_vel_dist(config.reduce_max_vel_sim);
    this->planner_.set_marge_in_distance(config.marge_in_distance);
    this->planner_.set_marge_angle_companion(config.marge_angle_companion);
    debug_real_test_companion_robot_=config.debug_robot_ROS_Info;
    this->planner_.set_mode_debug_cout(config.debug_robot_cout_library);
    this->planner_.set_mode_debug_file(config.debug_robot_file);
    this->planner_.set_debug_filename(config.debug_filename);
    this->planner_.set_externa_force_k_near_goal_akp(config.set_external_near_k);
    this->planner_.set_externa_force_k_far_goal_akp(config.set_external_far_k);
    this->planner_.set_ex_max_dist_to_near_goal_force_akp(config.ex_max_dist_to_near_goal_force);
    this->planner_.set_mode_step_near(config.mode_step_near);
    this->planner_.set_out_index_step(config.out_index_step);
    this->planner_.only_comp_people_vel_and_robot_poses(config.debug_only_robot_point_comp_person_point);
    this->planner_.set_person_radi_amp(config.person_radi_amp);
    this->planner_.set_obstacle_radi_amp(config.obstacle_radi_amp);
    this->planner_.set_ini_vel_to_increment_angle(config.ini_vel_to_increment_angle);
    */   // end companion config variables.
    this->scene_.get_akp_for_person_companion()->set_evaluate_costs_filename(config.evaluate_costs_filename);

    // get person goal ID
    //id_person_goal_=2;
    id_person_goal_=config.person_goal_id;  
    this->scene_.set_id_person_goal(config.person_goal_id);
    this->scene_.get_akp_for_person_companion()->set_id_person_goal(config.person_goal_id);
    //this->scene_.get_akp_for_person_companion()->set_id_person_goal(3);
	  this->robot_companion_person_name_=config.robot_sim_comp_pers_name;
    this->robot_companion_person_name_out_node_=config.other_person_of_group_comp_sim_name;
	  this->scene_.set_other_companion_person_id(config.id_person_other_person_of_group_companion);
    this->planner_mutex_exit();
  }

    id_person_goal_=config.person_goal_id;  
    this->scene_.set_id_person_goal(config.person_goal_id);
    this->scene_.get_akp_for_person_companion()->set_id_person_goal(config.person_goal_id);
	  this->scene_.set_other_companion_person_id(config.id_person_other_person_of_group_companion);
  /* Cscene_sim config */
	//ROS_INFO("         *******  Cscene_sim config update  *******\n\n");
  ini_person_theta=config.ini_theta_companion_person;
  simulation_mode_ = (AkpLocalPlanner::simulation_mode) config.simulation_mode;
	n_persons_ = config.number_persons;
	freeze_ = config.freeze;
	scene_.set_number_virtual_people( n_persons_ );
  scene_.set_robot_person_proximity_distance(config.proximity_distance_between_robot_and_person);
  scene_.set_proximity_distance_tolerance(config.proximity_distance_tolerance);
  scene_.set_id_person_companion(config.id_person_companion);
    this->person_companion_goal_force_marker       = config.person_companion_goal_force_marker;
    this->person_companion_person_forces_marker    = config.person_companion_person_forces_marker;
    this->person_companion_obstacles_forces_marker = config.person_companion_obstacles_forces_marker;
    this->person_companion_resultant_force_marker  = config.person_companion_resultant_force_marker;
    //ROS_INFO(" 1 algorithm config update \n");
    // person_companion_goal_providing_mode_ = (AkpLocalPlanner::Goal_providing_mode)config.person_companion_goal_providing_mode;
    // ROS_INFO(" 2 algorithm config update \n");
    //slicing_path_diff_orientation_ = config.slicing_diff_orientation;
    //ROS_INFO(" 3 algorithm config update \n");
  this->scene_.set_CsceneSim_group_go_to_interact_with_other_person(config.conf_bool_group_go_to_interact_with_other_person);
  this->robot_companion_person_name_=config.robot_sim_comp_pers_name;
  this->robot_companion_person_name_out_node_=config.other_person_of_group_comp_sim_name;

  this->scene_.set_less_companion_velocity(config.max_vel_comp_pers); //max_vel_comp_pers= maximum velocity person companion.
  this->scene_.get_akp_for_person_companion()->set_companion_angle_peopl_in_group(config.companion_angle_respect_people_in_group);
  this->scene_.get_akp_for_person_companion()->set_real_distance_between_people_of_group_(config.conf_real_distance_between_people_of_group);
  this->scene_.set_horitzon_time_CsceneSim(config.horizon_time);


}

//copied from goal_functions.cpp in base_local_planner
bool AkpLocalPlanner::transformGlobalPlan(const tf::TransformListener& tf, 
                                            const std::vector<geometry_msgs::PoseStamped>& global_plan, 
                                            const costmap_2d::Costmap2DROS& costmap, 
                                            const std::string& global_frame, 
                                            std::vector<geometry_msgs::PoseStamped>& transformed_plan)
{
  if(debug_antes_subgoals_entre_AKP_goals_){
    ROS_INFO("[DABO] AkpLocalPlanner::transformGlobalPlan!!!!");
  }

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

   //if(debug_antes_subgoals_entre_AKP_goals_){
   ROS_INFO("[DABO] AkpLocalPlanner::transformPose!!!");
  //}
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
  if(debug_goal_person_){
    ROS_INFO("[DABO] AkpLocalPlanner::publishPlan!!!");
  }
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



///////////////////////// Do goal new (ely) functions to allow companion person goals, not only rviz goals.


void AkpLocalPlanner::move_baseDone(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResultConstPtr& result) 
{ 


  //this->alg_.lock(); 
   if(debug_antes_subgoals_entre_AKP_goals_){
    ROS_INFO("AkpLocalPlanner::move_baseDone: %s", state.toString().c_str());
    }

  if(this->move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("moveRobot: Hooray, the base reached the goal!");
    //return 1;
    this->good_goal_=true;
  }
  else
  {
    ROS_WARN("moveRobot: Oops, the base failed to reach the goal for some reason!");
    //return 2;
    this->good_goal_=false;  
  } 

 this->isMoveBaseActive=false;
  //this->alg_.unlock(); 
} 

void AkpLocalPlanner::move_baseActive() 
{ 
//ROS_INFO("IN AkpLocalPlanner::move_baseActive: ");
  //this->alg_.lock(); 
  //ROS_INFO("TibiDaboFafAlgNode::move_baseActive: Goal just went active!"); 
  

  //this->alg_.unlock(); 
} 

void AkpLocalPlanner::move_baseFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) 
{ 
  if(debug_goal_person_){
  ROS_INFO("IN AkpLocalPlanner::move_baseFeedback: ");
  }
  //this->alg_.lock(); 
  bool feedback_is_ok = true; 
  if( !feedback_is_ok ) 
    this->move_base_client_.cancelGoal(); 
  //this->alg_.unlock(); 
}


/////////////  [action requests] //////////////////
bool AkpLocalPlanner::move_baseMakeActionRequest() 
{ 
  if(debug_goal_person_){
  ROS_INFO("IN AkpLocalPlanner::move_baseMakeActionRequest: ");
  }
  bool ok=false;
  //if(this->config.move_base)
  //{
    if(this->move_base_client_.isServerConnected())
    {
      //this->alg_.unlock();
move_base_goal_.target_pose.pose.orientation.z=1;

      this->move_base_client_.sendGoal(move_base_goal_, 
                  boost::bind(&AkpLocalPlanner::move_baseDone,     this, _1, _2), 
                  boost::bind(&AkpLocalPlanner::move_baseActive,   this), 
                  boost::bind(&AkpLocalPlanner::move_baseFeedback, this, _1));
      //this->alg_.lock();
      //ROS_INFO("AkpLocalPlanner::move_baseMakeActionRequest: Goal Sent!");
      this->isMoveBaseActive=true;
      ok=true;
  if(debug_goal_person_){
  ROS_INFO("AkpLocalPlanner::move_baseMakeActionRequest: Goal Sent! move_base_goal_.target_pose.pose.position.x=%f; y=%f, %f",move_base_goal_.target_pose.pose.position.x,move_base_goal_.target_pose.pose.position.y,move_base_goal_.target_pose.pose.orientation.z*180/3.14);
  }
     // ROS_INFO("Goal sent. Waiting for result...");
 //     this->move_base_client_.waitForResult(); //Alternative: ac.waitForResult(ros::Duration(20.0)); //wait 20 seconds. para esteprar cierto tiempo... puedo marcarle el tiempo.
//      ROS_INFO("Current State: %s\n", this->move_base_client_.getState().toString().c_str());
//      if(this->move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//      {
//        ROS_INFO("moveRobot: Hooray, the base reached the goal!");
//        //return 1;
//        ok=true;
//      }
//      else
//      {
//        ROS_WARN("moveRobot: Oops, the base failed to reach the goal for some reason!");
//        //return 2;
//        ok=false;  
//      } 
    
    
    }
    else
    {
      ROS_ERROR("AkpLocalPlanner::move_baseMakeActionRequest: move_base server not connected");
      ok=false;
    }
  //}
  //else
  //{
   // this->cancelMoveBaseIfActive();
   // ok=true;
  //}

  return ok;
}

bool AkpLocalPlanner::doGoal() //(std::vector<double> goal)
{

// para lanzar el goal: this->doGoal();

Sdestination best_person_dest=this->scene_.get_person_companion_person_abstract().get_best_dest();
// goal of the person companion
  if(debug_goal_person_){
   ROS_INFO("INNNN AkpLocalPlanner::doGoal:");
  }
 //this->getAndPublishImage(); //AG150730: assure that the image is published
  //if((this->debug2)||(this->config_.debug)){
 // ROS_INFO("AkpLocalPlanner::doGoal: Goal is x=%f, y=%f",best_person_dest.x,best_person_dest.y);
  //}
  this->move_base_goal_.target_pose.pose.position.x = best_person_dest.x; //goal[0];
  this->move_base_goal_.target_pose.pose.position.y = best_person_dest.y; //goal[1];
  double theta = atan2(best_person_dest.y - this->robot_pose_.y , best_person_dest.x - this->robot_pose_.x);
  this->move_base_goal_.target_pose.pose.orientation.x = 0;//goal.get_robot_orient_x();
	this->move_base_goal_.target_pose.pose.orientation.y = 0;//goal.get_robot_orient_y();
	this->move_base_goal_.target_pose.pose.orientation.z = theta;//goal.get_robot_orient_z();
	this->move_base_goal_.target_pose.pose.orientation.w = 0;//goal.get_robot_orient_w();
  //ROS_INFO("Next robot goal: (%f,%f) %fº",goal[0],goal[1],goal[2]*180.0/M_PI); 

  this->move_base_goal_.target_pose.header.stamp    = ros::Time::now();
  this->move_base_goal_.target_pose.header.frame_id = "map";// == desde que frame_id mandas el goal, cambiar si va mal!. this->config.faf_frame; (lo que habia antes)
//ROS_INFO("OUTTT AkpLocalPlanner::doGoal:");



    next_goal_marker_.pose.position.x = best_person_dest.x;
    next_goal_marker_.pose.position.y = best_person_dest.y;
    next_goal_marker_.id = 17;
    this->personCompanion_MarkerArray_msg_.markers.push_back( next_goal_marker_ ); 

  if(debug_goal_person_){  
    ROS_INFO("next_goal_marker_: (%f,%f)",best_person_dest.x,best_person_dest.y); 
  }

  if(this->move_baseMakeActionRequest())
    return true;
  else
    return false;


 /* //this->getAndPublishImage(); //AG150730: assure that the image is published
  //if((this->debug2)||(this->config_.debug)){
  ROS_INFO("TibiDaboFafAlgNode::doGoal: Goal is x=%f, y=%f",goal.x,goal.y);
  //}
  this->move_base_goal_.target_pose.pose.position.x = goal.x; //goal[0];
  this->move_base_goal_.target_pose.pose.position.y = goal.y; //goal[1];
  double theta = atan2(robot_goal_.y - this->robot_pose_.y , robot_goal_.x - this->robot_pose_.x);
  this->move_base_goal_.target_pose.pose.orientation.x = 0;//goal.get_robot_orient_x();
	this->move_base_goal_.target_pose.pose.orientation.y = 0;//goal.get_robot_orient_y();
	this->move_base_goal_.target_pose.pose.orientation.z = theta;//goal.get_robot_orient_z();
	this->move_base_goal_.target_pose.pose.orientation.w = 0;//goal.get_robot_orient_w();
  //ROS_INFO("Next robot goal: (%f,%f) %fº",goal[0],goal[1],goal[2]*180.0/M_PI); 

  this->move_base_goal_.target_pose.header.stamp    = ros::Time::now();
  this->move_base_goal_.target_pose.header.frame_id = "map";// == desde que frame_id mandas el goal, cambiar si va mal!. this->config.faf_frame; (lo que habia antes)
  if(this->move_baseMakeActionRequest())
    return true;
  else
    return false;
*/
} 

/* Cscene_sim finctions (ini) */

void AkpLocalPlanner::init_sim()
{

 // this->fixed_frame = "/map"; // (ely) variable to include fake laser (fake obstacles) in people simulation ya esta en el akp

  scene_.set_dt( 0.2 );
  this->scene_.get_akp_for_person_companion()->set_dt(0.2);
  //this->scene_.get_akp_for_person_companion_2groupPers()->set_dt(0.2);
	//this->public_node_handle_.getParam("simulation_mode", simulation_mode_);
  this->public_node_handle_.getParam("number_persons", n_persons_);
  this->public_node_handle_.getParam("force_map_path", force_map_path_);
  this->public_node_handle_.getParam("destination_map_path", destination_map_path_);
  this->public_node_handle_.getParam("remove_targets", remove_targets_);
  this->public_node_handle_.getParam("freeze", freeze_);
  this->public_node_handle_.getParam("robot", robot_);

	scene_.set_number_virtual_people( n_persons_ );
  scene_.set_remove_targets( remove_targets_ );
	peopleTrackingArray_msg_.header.frame_id = "/map";
  destinationsOfTracksArray_msg_.header.frame_id = "/map";

  ROS_INFO("AkpLocalPlanner::init_sim:  antes scene_.read_destination_map2!!!");
  //read destinations
	if ( !scene_.read_destination_map2(  destination_map_path_.c_str() ) )
	{
		ROS_ERROR("AkpLocalPlanner::init_sim(read_destination_map2): Could not read map destinations file !!!");
	}
  else{
		ROS_INFO("AkpLocalPlanner:::init_sim(read_destination_map2): read destinations map file : SUCCESS!!!");
	}
	if ( !scene_.get_akp_for_person_companion()->read_destination_map2(  destination_map_path_.c_str() ) )
	{
		ROS_ERROR("AkpLocalPlanner::init_sim(read_destination_map2): Could not read map destinations file for akp plannin of companion person!!!");
	}
  else{
		ROS_INFO("AkpLocalPlanner::init_sim(read_destination_map2): read destinations map file for akp plannin of companion person: SUCCESS!!! %s", destination_map_path_.c_str());
	}
ROS_INFO("AkpLocalPlanner::init_sim:  despues scene_.read_destination_map2!!!");
  //read force map
	if ( !scene_.read_force_map(  force_map_path_.c_str() ) )
	{
		ROS_ERROR("AkpLocalPlanner::init_sim: Could not read map force file !!!");
	}
  else{
		ROS_INFO("AkpLocalPlanner::init_sim: read map force file : SUCCESS!!!");
	}

  if ( !scene_.get_akp_for_person_companion()->read_force_map(  force_map_path_.c_str() ) )
	{
		ROS_ERROR("AkpLocalPlanner::init_sim: Could not read map force file for akp plannin of companion person !!!");
	}
  else{
		ROS_INFO("AkpLocalPlanner::init_sim: read map force file for akp plannin of companion person: SUCCESS!!!");
	}
	//false = no destroy simulated persons
	//scene_.set_remove_targets(true);

	//target marker
	track_marker_.header.frame_id = "/map";
	track_marker_.ns = "tracks";
	track_marker_.type = visualization_msgs::Marker::CYLINDER;
	track_marker_.action = visualization_msgs::Marker::ADD;
	track_marker_.lifetime = ros::Duration(1.0f);
	track_marker_.scale.x = 0.5;
	track_marker_.scale.y = 0.5;
	track_marker_.scale.z = 0.6;
	track_marker_.color.a = 0.6;
	track_marker_.color.r = 0.0;
	track_marker_.color.g = 1.0;
	track_marker_.color.b = 0.0;
	track_marker_.pose.position.z = 0.3;

	id_marker_.header.frame_id = "/map";
	id_marker_.ns = "ids";
	id_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	id_marker_.action = visualization_msgs::Marker::ADD;
	id_marker_.lifetime = ros::Duration(1.0f);
	id_marker_.scale.z = 0.5;
	id_marker_.pose.position.z = 1.0;
	id_marker_.color.a = 0.5;
	id_marker_.color.r = 0.0;
	id_marker_.color.g = 0.0;
	id_marker_.color.b = 0.0;
	ROS_INFO("AkpLocalPlanner::init_sim:  SUCCESS!!!");

  init_person_companion_markers();

} 


bool AkpLocalPlanner::do_Cscene_sim(Spose& pose_person_companion)
{

peopleTrackingArray_msg_.header.stamp = ros::Time::now();
destinationsOfTracksArray_msg_.header.stamp = ros::Time::now();

  //for every new trajectory
	iri_perception_msgs::detection person;
  iri_perception_msgs::detection destination_act;

  if(debug_person_companion_){
    if(freeze_){
      ROS_INFO(" IN AkpLocalPlanner::do_Cscene_sim(); freeze_=true");
    }else{
      ROS_INFO(" IN AkpLocalPlanner::do_Cscene_sim(); freeze_=false");
    }
  } 

  if(debug_person_companion_){
    ROS_INFO(" 2 IN AkpLocalPlanner::do_Cscene_sim()");
  }
  //ROS_INFO("(dabo) AkpLocalPlanner::mainNodeThread: despues if(tf_exists)");  
	std::vector<SdetectionObservation> obs_scene;
   //ROS_INFO("(dabo) AkpLocalPlanner::mainNodeThread: antes obs_scene.push_back");  
	obs_scene.push_back( SdetectionObservation(0, ros::Time::now().toSec() ));//void observation, just for the timestamp
  if(debug_person_companion_){
    ROS_INFO(" 3 IN AkpLocalPlanner::do_Cscene_sim()");
  }
	//scene_.update_scene( obs_scene );
  if(debug_person_companion_){
    ROS_INFO( "4 IN AkpLocalPlanner::do_Cscene_sim(); antes !!! scene_.update_scene_companion_simulation");
  }
  // Spose best_next_pose;
  //scene_.update_scene_companion_simulation(obs_scene,ini_companion_); // OJO!!! modificado para hacer grupos de personas.
  bool robot_plan_succed=false;
  //ROS_INFO(" 4.1 IN AkpLocalPlanner::do_Cscene_sim()");
  robot_plan_succed=scene_.update_scene_companion_simulation_akp_person_companion(obs_scene,ini_companion_,ini_person_theta,pose_person_companion); //modificado grupos + person companion akp
  //ROS_INFO(" 5 IN AkpLocalPlanner::do_Cscene_sim()");
  new_person_companion_position_=this->scene_.get_person_companion_person_abstract();

  if(group_simulation_now_){
    new_person_companion_position_2groupPers_=this->scene_.get_person_companion_person_abstract_2group_person();
	//SpointV get_cur_poin=new_person_companion_position_2groupPers_.get_current_pointV();
	//this->robot_pose_2personGroup_=Spose(get_cur_poin.x,get_cur_poin.y,get_cur_poin.time_stamp, atan(get_cur_poin.vy/get_cur_poin.vx), sqrt((get_cur_poin.vx)*(get_cur_poin.vx)+(get_cur_poin.vy)*(get_cur_poin.vy)));
  } 

 
	//robot_pose_2personGroup_=this->scene_.get_person_companion_person_abstract_2group_person()
  //if(debug_person_companion_){
  //  ROS_INFO(" 6 IN AkpLocalPlanner::do_Cscene_sim()");
  //}
 // Cperson_abstract::companion_reactive reactive_;
 // ROS_INFO( "antes !!! de get_akp_for_person_companion()->person_companion_plan_companion(best_next_pose,reactive_);");
 // this->get_akp_for_person_companion()->person_companion_plan_companion(best_next_pose,reactive_);
 // robot_pose_=best_next_pose;
//  ROS_INFO( "despues !!! scene_.update_scene_companion_simulation");
  ini_companion_=false; // despues de inicializar las personas lo pones a falso.
  obs.clear();
  //publish data
  const std::list<Cperson_abstract *>* person_list = scene_.get_scene( ); // Todo, puede que esta tenga que ser ahora la interna, no esta...
  const std::list<Cperson_abstract *>* person_list2 = scene_.get_akp_for_person_companion()->get_scene( );

  if(group_simulation_now_){
    person_list=person_list2; // saco verdaderamente la person list que usa el robot (person-companion) para calcular el planning.
  }

  if(debug_person_companion_){
    ROS_INFO("2");
  }
  if( !freeze_ )
  {
    if(debug_person_companion_){
      if(person_list->empty()){
        ROS_INFO("in freze; person_list->empty()=true; person_list->size()=%d",person_list->size());
      }else{
        ROS_INFO("in freze; person_list->empty()=false; person_list->size()=%d",person_list->size());
      }
      if(person_list2->empty()){
        ROS_INFO("in freze; person_list2->empty()=true; person_list->size()=%d",person_list->size());
      }else{
        ROS_INFO("in freze; person_list2->empty()=false; person_list->size()=%d",person_list->size());
      }
    }

// ROS_INFO(" 7 IN AkpLocalPlanner::do_Cscene_sim()");

    peopleTrackingArray_msg_.detection.clear();
    destinationsOfTracksArray_msg_.detection.clear();
    for( std::list<Cperson_abstract*>::const_iterator iit = person_list->begin();
          iit!=person_list->end(); iit++ )
    {
		//if((*iit)->get_id()!=5){
		  person.id = (*iit)->get_id();
		  person.position.x = (*iit)->get_current_pointV().x;
		  person.position.y = (*iit)->get_current_pointV().y;
		  person.velocity.x = (*iit)->get_current_pointV().vx;
		  person.velocity.y = (*iit)->get_current_pointV().vy;
      //covariances conversion from 4x4 to 6x6
    	person.covariances[0] = (*iit)->get_current_pointV().cov[0];//x x
    	person.covariances[1] = (*iit)->get_current_pointV().cov[1];//x y
    	person.covariances[3] = (*iit)->get_current_pointV().cov[2];//x vx
    	person.covariances[4] = (*iit)->get_current_pointV().cov[3];//x vy
    	person.covariances[6] = (*iit)->get_current_pointV().cov[4];//x y
    	person.covariances[7] = (*iit)->get_current_pointV().cov[5];//y y
    	person.covariances[9] = (*iit)->get_current_pointV().cov[6];//y vx
    	person.covariances[10] = (*iit)->get_current_pointV().cov[7];//y vy
    	person.covariances[18] = (*iit)->get_current_pointV().cov[8];//vx x
    	person.covariances[19] = (*iit)->get_current_pointV().cov[9];//vx y
    	person.covariances[21] = (*iit)->get_current_pointV().cov[10];//vx vx
    	person.covariances[22] = (*iit)->get_current_pointV().cov[11];//vx vy
    	person.covariances[24] = (*iit)->get_current_pointV().cov[12];//vy x
    	person.covariances[25] = (*iit)->get_current_pointV().cov[13];//vy y
    	person.covariances[27] = (*iit)->get_current_pointV().cov[14];//vy vx
    	person.covariances[28] = (*iit)->get_current_pointV().cov[15];//vy vy
      
   // }

   // std::cout << " 333333333333333333333 ANTES output tracks other person companion "<< std::endl;
    if(group_simulation_now_){
        if(person.id<11){ // TODO: insertar vector que te diga de que robot es de los dos. Quizas vector de strings con el nombre del robot-person-companion
                          // al que pertenece ese track. I así se podría diferenciar para N personas que acompañaran al robot.
          peopleTrackingArray_msg_.detection.push_back(person);
          //std::cout << " 333333333333333333333 (output track) person.id="<<person.id<<"; person.position.x="<<person.position.x<<"; person.position.y="<<person.position.y<< std::endl;
          destination_act.id = (*iit)->get_id(); // destination de las personas de esta person companion, para pasarselas al otro(robot==person companion)
            // usamos como id, el id de este track de la persona y como posicion la x e y de la best_dest actual.
		      destination_act.position.x = (*iit)->get_best_dest().x;
		      destination_act.position.y = (*iit)->get_best_dest().y;
          destinationsOfTracksArray_msg_.detection.push_back(destination_act);
        }

    }else{
      peopleTrackingArray_msg_.detection.push_back(person);
    }
		 
 
     // (*iit)->print();

      std::vector<double> cov;
      cov.resize(16,0.0);
      cov[0] = (*iit)->get_current_pointV().cov[0];//x x
      cov[1] = (*iit)->get_current_pointV().cov[1];//x y
      cov[2] = (*iit)->get_current_pointV().cov[2];//x vx
      cov[3] = (*iit)->get_current_pointV().cov[3];//x vy
      cov[4] = (*iit)->get_current_pointV().cov[4];//x y
      cov[5] = (*iit)->get_current_pointV().cov[5];//y y
      cov[6] = (*iit)->get_current_pointV().cov[6];//y vx
      cov[7] = (*iit)->get_current_pointV().cov[7];//y vy
      cov[8] = (*iit)->get_current_pointV().cov[8];//vx x
      cov[9] = (*iit)->get_current_pointV().cov[9];//vx y
      cov[10] = (*iit)->get_current_pointV().cov[10];//vx vx
      cov[11] = (*iit)->get_current_pointV().cov[11];//vx vy
      cov[12] = (*iit)->get_current_pointV().cov[12];//vy x
      cov[13] = (*iit)->get_current_pointV().cov[13];//vy y
      cov[14] = (*iit)->get_current_pointV().cov[14];//vy vx
      cov[15] = (*iit)->get_current_pointV().cov[15];//vy vy
      double x=(*iit)->get_current_pointV().x;
      double y=(*iit)->get_current_pointV().y;
      double vx=(*iit)->get_current_pointV().vx;
      double vy=(*iit)->get_current_pointV().vy;
      int id=(*iit)->get_id();
      obs.push_back(SdetectionObservation(id, ros::Time::now().toSec(),x, y ,vx, vy, cov));
	  }
//ROS_INFO(" 8 IN AkpLocalPlanner::do_Cscene_sim()");
    /* INI person companion track (for publish) */

    person.id = scene_.get_id_person_companion();
	  person.position.x = new_person_companion_position_.get_current_pointV().x;
	  person.position.y = new_person_companion_position_.get_current_pointV().y;
	  person.velocity.x = new_person_companion_position_.get_current_pointV().vx;
	  person.velocity.y = new_person_companion_position_.get_current_pointV().vy;
 //ROS_INFO("EEEEEEEEEEE (nodo PERSON sim) VELOCITY published vx=%f ; vy=%f. ",person.velocity.x,person.velocity.y);
     // std::cout << "   EEEEEEEEe    (nodo PERSON sim) VELOCITY published vx="<<person.velocity.x<<"; vy="<<person.velocity.y<< std::endl;
    //covariances conversion from 4x4 to 6x6
    person.covariances[0] = new_person_companion_position_.get_current_pointV().cov[0];//x x
    person.covariances[1] = new_person_companion_position_.get_current_pointV().cov[1];//x y
    person.covariances[3] = new_person_companion_position_.get_current_pointV().cov[2];//x vx
    person.covariances[4] = new_person_companion_position_.get_current_pointV().cov[3];//x vy
    person.covariances[6] = new_person_companion_position_.get_current_pointV().cov[4];//x y
    person.covariances[7] = new_person_companion_position_.get_current_pointV().cov[5];//y y
    person.covariances[9] = new_person_companion_position_.get_current_pointV().cov[6];//y vx
    person.covariances[10] = new_person_companion_position_.get_current_pointV().cov[7];//y vy
    person.covariances[18] = new_person_companion_position_.get_current_pointV().cov[8];//vx x
    person.covariances[19] = new_person_companion_position_.get_current_pointV().cov[9];//vx y
    person.covariances[21] = new_person_companion_position_.get_current_pointV().cov[10];//vx vx
    person.covariances[22] = new_person_companion_position_.get_current_pointV().cov[11];//vx vy
    person.covariances[24] = new_person_companion_position_.get_current_pointV().cov[12];//vy x
    person.covariances[25] = new_person_companion_position_.get_current_pointV().cov[13];//vy y
    person.covariances[27] = new_person_companion_position_.get_current_pointV().cov[14];//vy vx
    person.covariances[28] = new_person_companion_position_.get_current_pointV().cov[15];//vy vy
		peopleTrackingArray_msg_.detection.push_back(person);
    /* FIN person companion track (for publish) */


    /* INI person companion 2 track (for publish) */
    /*if(group_simulation_now_){
       person.id = scene_.get_id_person_companion_2group_person();
       person.position.x = new_person_companion_position_2groupPers_.get_current_pointV().x;
       person.position.y = new_person_companion_position_2groupPers_.get_current_pointV().y;
       person.velocity.x = new_person_companion_position_2groupPers_.get_current_pointV().vx;
       person.velocity.y = new_person_companion_position_2groupPers_.get_current_pointV().vy;
       //ROS_INFO("EEEEEEEEEEE (nodo PERSON sim) VELOCITY published vx=%f ; vy=%f. ",person.velocity.x,person.velocity.y);
       // std::cout << "   EEEEEEEEe    (nodo PERSON sim) VELOCITY published vx="<<person.velocity.x<<"; vy="<<person.velocity.y<< std::endl;
       //covariances conversion from 4x4 to 6x6
       person.covariances[0] = new_person_companion_position_2groupPers_.get_current_pointV().cov[0];//x x
       person.covariances[1] = new_person_companion_position_2groupPers_.get_current_pointV().cov[1];//x y
       person.covariances[3] = new_person_companion_position_2groupPers_.get_current_pointV().cov[2];//x vx
       person.covariances[4] = new_person_companion_position_2groupPers_.get_current_pointV().cov[3];//x vy
       person.covariances[6] = new_person_companion_position_2groupPers_.get_current_pointV().cov[4];//x y
       person.covariances[7] = new_person_companion_position_2groupPers_.get_current_pointV().cov[5];//y y
       person.covariances[9] = new_person_companion_position_2groupPers_.get_current_pointV().cov[6];//y vx
       person.covariances[10] = new_person_companion_position_2groupPers_.get_current_pointV().cov[7];//y vy
       person.covariances[18] = new_person_companion_position_2groupPers_.get_current_pointV().cov[8];//vx x
       person.covariances[19] = new_person_companion_position_2groupPers_.get_current_pointV().cov[9];//vx y
       person.covariances[21] = new_person_companion_position_2groupPers_.get_current_pointV().cov[10];//vx vx
       person.covariances[22] = new_person_companion_position_2groupPers_.get_current_pointV().cov[11];//vx vy
       person.covariances[24] = new_person_companion_position_2groupPers_.get_current_pointV().cov[12];//vy x
       person.covariances[25] = new_person_companion_position_2groupPers_.get_current_pointV().cov[13];//vy y
       person.covariances[27] = new_person_companion_position_2groupPers_.get_current_pointV().cov[14];//vy vx
       person.covariances[28] = new_person_companion_position_2groupPers_.get_current_pointV().cov[15];//vy vy
       peopleTrackingArray_msg_.detection.push_back(person);
    } */
    /* FIN person companion 2 track (for publish) */


//ROS_INFO(" 9 IN AkpLocalPlanner::do_Cscene_sim()");
  if(debug_goal_person_){
   std::cout << "(nodo) [do_Cscene_sim] new_person_companion_position_:"<< std::endl;
  new_person_companion_position_.print();
  }
//ROS_INFO(" 10 IN AkpLocalPlanner::do_Cscene_sim()");
    if(debug_person_companion_){
      std::cout << "(nodo) std::vector<SdetectionObservation> obs:"<< std::endl;
	    for( unsigned int ty = 0 ; ty< this->obs.size() ; ty++)
	    {
			  obs[ty].print();
	    }
    }
//ROS_INFO(" 11 IN AkpLocalPlanner::do_Cscene_sim()");
   // bool have_companion_person=true;
	 // scene_.get_akp_for_person_companion()->update_scene(obs,have_companion_person, true);

    if(debug_person_companion_){
      ROS_INFO(" 6.2 (print1) in");
      scene_.get_akp_for_person_companion()->print();
    }
    
  }
  //ROS_INFO(" 11.3 (print2) out");
  //scene_.get_akp_for_person_companion()->print2();
// ROS_INFO(" 12 IN AkpLocalPlanner::do_Cscene_sim()"); 
  if(debug_person_companion_){
    ROS_INFO(" 7 IN AkpLocalPlanner::do_Cscene_sim()");
  }
//ROS_INFO(" 13 IN AkpLocalPlanner::do_Cscene_sim()");
	if( vis_mode_ && !freeze_)
	{
		MarkerArray_msg_.markers.clear();
		track_marker_.header.stamp =   peopleTrackingArray_msg_.header.stamp;
		id_marker_.header.stamp =   peopleTrackingArray_msg_.header.stamp;
    // draw targets
//ROS_INFO(" 14 IN AkpLocalPlanner::do_Cscene_sim()");

    for( std::list<Cperson_abstract*>::const_iterator iit = person_list->begin();
          iit!=person_list->end(); iit++ )
    {
			track_marker_.id = (*iit)->get_id();
			track_marker_.pose.position.x = (*iit)->get_current_pointV().x;
			track_marker_.pose.position.y = (*iit)->get_current_pointV().y;
      if((*iit)->get_id()==2){
        track_marker_.color.a = 0.5;
        track_marker_.color.r = 1.0;
        track_marker_.color.g = 0.0;
        track_marker_.color.b = 0.0;
      }else{
        track_marker_.color.a = 1.0;
        track_marker_.color.r = 0.0;
        track_marker_.color.g = 1.0;
        track_marker_.color.b = 0.0;
      }      
      
     // ROS_INFO(" AkpLocalPlanner::mainNodeThread; track_marker_=%d; track_marker_.pose.position.x=%f; track_marker_.pose.position.y=%f",track_marker_.id,track_marker_.pose.position.x,track_marker_.pose.position.y);

			MarkerArray_msg_.markers.push_back(track_marker_);
			id_marker_.id = (*iit)->get_id();
			id_marker_.pose.position.x = (*iit)->get_current_pointV().x;
			id_marker_.pose.position.y = (*iit)->get_current_pointV().y;
			std::ostringstream target_id;
			target_id << id_marker_.id;
			id_marker_.text = target_id.str();
			MarkerArray_msg_.markers.push_back(id_marker_);
		}
//ROS_INFO(" 15 IN AkpLocalPlanner::do_Cscene_sim()");
    	/* Person companion marker (track) */
      	track_marker_.id = scene_.get_id_person_companion(); // get_companion id...
	      track_marker_.pose.position.x =new_person_companion_position_.get_current_pointV().x;
	      track_marker_.pose.position.y = new_person_companion_position_.get_current_pointV().y;
      	track_marker_.color.a = 1.0;
      	track_marker_.color.r = 0.0;
       	track_marker_.color.g = 0.5;
        track_marker_.color.b = 0.6;
     // ROS_INFO(" AkpLocalPlanner::mainNodeThread; track_marker_=%d; track_marker_.pose.position.x=%f; track_marker_.pose.position.y=%f",track_marker_.id,track_marker_.pose.position.x,track_marker_.pose.position.y);

	MarkerArray_msg_.markers.push_back(track_marker_);
	id_marker_.id = scene_.get_id_person_companion(); // get_companion id...
	id_marker_.pose.position.x = new_person_companion_position_.get_current_pointV().x;
	id_marker_.pose.position.y = new_person_companion_position_.get_current_pointV().y;
	std::ostringstream target_id;
	target_id << id_marker_.id;
	id_marker_.text = target_id.str();
	MarkerArray_msg_.markers.push_back(id_marker_);
 /* FIN Person companion marker (track) */


    	/* Person companion 2 marker (track) */
     /* 	track_marker_.id = scene_.get_id_person_companion_2group_person(); // get_companion id...
	track_marker_.pose.position.x =	new_person_companion_position_2groupPers_.get_current_pointV().x;
	track_marker_.pose.position.y = new_person_companion_position_2groupPers_.get_current_pointV().y;
      	track_marker_.color.a = 1.0;
      	track_marker_.color.r = 0.0;
       	track_marker_.color.g = 0.5;
        track_marker_.color.b = 0.6;
     // ROS_INFO(" AkpLocalPlanner::mainNodeThread; track_marker_=%d; track_marker_.pose.position.x=%f; track_marker_.pose.position.y=%f",track_marker_.id,track_marker_.pose.position.x,track_marker_.pose.position.y);

	MarkerArray_msg_.markers.push_back(track_marker_);
	id_marker_.id = scene_.get_id_person_companion_2group_person(); // get_companion id...
	id_marker_.pose.position.x = new_person_companion_position_2groupPers_.get_current_pointV().x;
	id_marker_.pose.position.y = new_person_companion_position_2groupPers_.get_current_pointV().y;
	//std::ostringstream target_id;
	target_id << id_marker_.id;
	id_marker_.text = target_id.str();
	MarkerArray_msg_.markers.push_back(id_marker_);*/
 /* FIN Person companion marker (track) */


    //draw robot if any  (cambio ely, NO quiero un marker verde en el robot, mientras simulo el akp!!!!)
   /* if (  scene_.get_robot() != NULL )
    {
		  track_marker_.id = 0;
    	track_marker_.ns = "robot";
		  track_marker_.pose.position.x = scene_.get_robot()->get_current_pointV().x;
		  track_marker_.pose.position.y = scene_.get_robot()->get_current_pointV().y;
		  MarkerArray_msg_.markers.push_back(track_marker_);
    	track_marker_.ns = "tracks";
    }*/
//ROS_INFO(" 16 IN AkpLocalPlanner::do_Cscene_sim()");
    if(debug_person_companion_){
      ROS_INFO(" 8 IN AkpLocalPlanner::do_Cscene_sim()");
    }
//ROS_INFO(" 17 IN AkpLocalPlanner::do_Cscene_sim()");
		this->tracksMarkers_publisher_.publish(this->MarkerArray_msg_);

	}
  
   // slice_plan( );
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
// ROS_INFO(" FIN AkpLocalPlanner::mainNodeThread");
//ROS_INFO(" 17 IN AkpLocalPlanner::do_Cscene_sim()");
  this->personCompanion_MarkerArray_msg_.markers.clear();
  init_person_companion_markers();
 	
  Sdestination path_goal=this->scene_.CsceneSim_get_goal_path();
  Sdestination companion_goal=this->scene_.CsceneSim_get_goal_companion();
  // rellegnar dos markers espefa y publicarlos! para ver los goals a donde va la person companion.


  goal_path_marker_.pose.position.x = path_goal.x;
  goal_path_marker_.pose.position.y = path_goal.y; 
  this->personCompanion_MarkerArray_msg_.markers.push_back( goal_path_marker_ );

  goal_companion_marker_.pose.position.x = companion_goal.x;
  goal_companion_marker_.pose.position.y = companion_goal.y;
  this->personCompanion_MarkerArray_msg_.markers.push_back( goal_companion_marker_ );

  //ROS_INFO(" goal_path_marker_.x=%f; goal_path_marker_.y=%f;",goal_path_marker_.pose.position.x,goal_path_marker_.pose.position.y);
  //ROS_INFO(" goal_companion_marker_.x=%f; goal_companion_marker_.y=%f;",goal_companion_marker_.pose.position.x,goal_companion_marker_.pose.position.y);

  fill_scene_markers();
  fill_best_path_2d();
  fill_planning_markers_2d();  
  fill_laser_obstacles();
  fill_forces_markers();
//ROS_INFO(" 18 IN AkpLocalPlanner::do_Cscene_sim()");
  if( vis_mode_ > 1 ) //normal mode 2d features
  {
    //fill_planning_markers_2d();  
    //fill_people_prediction_markers_2d();
    //fill_forces_markers();
    //fill_laser_obstacles();
  }
  if ( vis_mode_ > 2 )//super mode 3d features
  {
  fill_people_prediction_markers_3d();
  //fill_planning_markers_3d();
  }
  
  //ROS_INFO(" 19 IN AkpLocalPlanner::do_Cscene_sim()");
  this->personCompanionAkpMarkers_publisher_.publish(this->personCompanion_MarkerArray_msg_);
  this->tracks_publisher_.publish(this->peopleTrackingArray_msg_);
  this->destinations_of_tracks_publisher_.publish(this->destinationsOfTracksArray_msg_);
  if(debug_person_companion_){
    ROS_INFO(" 9 FIN IN AkpLocalPlanner::do_Cscene_sim()");
  }


  return robot_plan_succed;
  //this->alg_.unlock();
}



void AkpLocalPlanner::init_person_companion_markers()
{


  // ROS_INFO( " 1 init_person_companion_markers() ");
  //target marker. Predicts the trajectories of people
  personCompanion_pred_traj_marker_.header.frame_id = this->fixed_frame;
  personCompanion_pred_traj_marker_.ns = "pred";
  personCompanion_pred_traj_marker_.type = visualization_msgs::Marker::CYLINDER;
  personCompanion_pred_traj_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_pred_traj_marker_.lifetime = ros::Duration(1.23f);
  // ROS_INFO( " 1 init_person_companion_markers(), %f ", scene_.get_akp_for_person_companion()->get_dt());
  personCompanion_pred_traj_marker_.scale.z = scene_.get_akp_for_person_companion()->get_dt();
  // ROS_INFO( " 1.2 init_person_companion_markers(), %f ", scene_.get_akp_for_person_companion()->get_dt());
  personCompanion_pred_traj_marker_.color.a = 0.05;
  personCompanion_pred_traj_marker_.color.r = 0.3;
  personCompanion_pred_traj_marker_.color.g = 1.0;
  personCompanion_pred_traj_marker_.color.b = 0.3;
  personCompanion_pred_traj_marker_.pose.orientation.w = 1.0;
  // ROS_INFO( " 2 init_person_companion_markers() ");
  //target marker of the best path, projected in 2d
  personCompanion_pred_traj2d_marker_.header.frame_id = this->fixed_frame;
  personCompanion_pred_traj2d_marker_.ns = "pred2d";
  personCompanion_pred_traj2d_marker_.type = visualization_msgs::Marker::CYLINDER;
  personCompanion_pred_traj2d_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_pred_traj2d_marker_.lifetime = ros::Duration(1.23f);
  personCompanion_pred_traj2d_marker_.scale.z = 0.05;
  personCompanion_pred_traj_marker_.color.a = 0.4;
  personCompanion_pred_traj_marker_.color.r = 0.3;
  personCompanion_pred_traj_marker_.color.g = 1.0;
  personCompanion_pred_traj_marker_.color.b = 0.3;
  personCompanion_pred_traj_marker_.pose.orientation.w = 1.0;
  personCompanion_pred_traj2d_marker_.pose.position.z = 0.025;
  // ROS_INFO( " 3 init_person_companion_markers() ");
  //cylinder marker, for destinations and goals
  personCompanion_cylinder_marker_.header.frame_id = this->fixed_frame;
  personCompanion_cylinder_marker_.ns = "scene";
  personCompanion_cylinder_marker_.type = visualization_msgs::Marker::CYLINDER;
  personCompanion_cylinder_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_cylinder_marker_.lifetime = ros::Duration(1.3f);
  personCompanion_cylinder_marker_.color.a = 1.0;
  personCompanion_cylinder_marker_.color.r = 0.8;// 0.3
  personCompanion_cylinder_marker_.color.g = 0.1;// 0.1
  personCompanion_cylinder_marker_.color.b = 0.8;// 0.8
  personCompanion_cylinder_marker_.scale.x = 0.7;
  personCompanion_cylinder_marker_.scale.y = 0.7;
  personCompanion_cylinder_marker_.scale.z = 1.2;
  personCompanion_cylinder_marker_.pose.position.z = 0.6;
  personCompanion_cylinder_marker_.pose.orientation.w = 1.0;
  // ROS_INFO( " 4 init_person_companion_markers() ");
  // robot goal marker, cylinders plot in the scene
  personCompanion_robot_goal_marker_.header.frame_id = this->fixed_frame;
  personCompanion_robot_goal_marker_.ns = "scene";
  personCompanion_robot_goal_marker_.type = visualization_msgs::Marker::CYLINDER;
  personCompanion_robot_goal_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_robot_goal_marker_.lifetime = ros::Duration(1.3f);
  personCompanion_robot_goal_marker_.color.a = 1.0;
  personCompanion_robot_goal_marker_.color.r = 0.0; // 0.9
  personCompanion_robot_goal_marker_.color.g = 0.0; // 0.3
  personCompanion_robot_goal_marker_.color.b = 0.0; // 0.3
  personCompanion_robot_goal_marker_.scale.x = 0.4;
  personCompanion_robot_goal_marker_.scale.y = 0.4;
  personCompanion_robot_goal_marker_.scale.z = 0.2;
  personCompanion_robot_goal_marker_.pose.position.z = 0.1;
  personCompanion_robot_goal_marker_.pose.orientation.w = 1.0;
  
  personCompanion_robot_subgoal_marker_ = personCompanion_robot_goal_marker_;
  personCompanion_robot_subgoal_marker_.color.r = 0.9;
  personCompanion_robot_subgoal_marker_.color.g = 0.6;
  personCompanion_robot_subgoal_marker_.color.b = 0.6;
  personCompanion_robot_subgoal_marker_.scale.x = 0.3;
  personCompanion_robot_subgoal_marker_.scale.y = 0.3;

  //ROS_INFO( " 5 init_person_companion_markers() ");
  // robot_real_goal_marker!!!

  personCompanion_robot_goal_marker2_.header.frame_id = this->fixed_frame;
  personCompanion_robot_goal_marker2_.ns = "scene";
  personCompanion_robot_goal_marker2_.type = visualization_msgs::Marker::CYLINDER;
  personCompanion_robot_goal_marker2_.action = visualization_msgs::Marker::ADD;
  personCompanion_robot_goal_marker2_.lifetime = ros::Duration(1.3f);
  personCompanion_robot_goal_marker2_.color.a = 1.0;
  personCompanion_robot_goal_marker2_.color.r = 0.0;
  personCompanion_robot_goal_marker2_.color.g = 0.0;
  personCompanion_robot_goal_marker2_.color.b = 1.0;
  personCompanion_robot_goal_marker2_.scale.x = 0.4;
  personCompanion_robot_goal_marker2_.scale.y = 0.4;
  personCompanion_robot_goal_marker2_.scale.z = 1.0;
  personCompanion_robot_goal_marker2_.pose.position.z = 0.1;
  personCompanion_robot_goal_marker2_.pose.orientation.w = 1.0;

  //ROS_INFO( " 6 init_person_companion_markers() ");
  // robot_real_goal_marker!!!

  personCompanion_robot_goal_marker3_.header.frame_id = this->fixed_frame;
  personCompanion_robot_goal_marker3_.ns = "scene";
  personCompanion_robot_goal_marker3_.type = visualization_msgs::Marker::CYLINDER;
  personCompanion_robot_goal_marker3_.action = visualization_msgs::Marker::ADD;
  personCompanion_robot_goal_marker3_.lifetime = ros::Duration(1.3f);
  personCompanion_robot_goal_marker3_.color.a = 0.0;
  personCompanion_robot_goal_marker3_.color.r = 0.5;
  personCompanion_robot_goal_marker3_.color.g = 1.0;
  personCompanion_robot_goal_marker3_.color.b = 1.0;
  personCompanion_robot_goal_marker3_.scale.x = 1.4;
  personCompanion_robot_goal_marker3_.scale.y = 1.4;
  personCompanion_robot_goal_marker3_.scale.z = 2.0;
  personCompanion_robot_goal_marker3_.pose.position.z = 0.1;
  personCompanion_robot_goal_marker3_.pose.orientation.w = 1.0;

  //ROS_INFO( " 7 init_person_companion_markers() ");
  // workspace marker
  personCompanion_workspace_marker_.header.frame_id = this->fixed_frame;
  personCompanion_workspace_marker_.ns = "scene";
  personCompanion_workspace_marker_.type = visualization_msgs::Marker::LINE_LIST;
  personCompanion_workspace_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_workspace_marker_.lifetime = ros::Duration(1.3f);
  personCompanion_workspace_marker_.scale.x = 0.05;
  personCompanion_workspace_marker_.color.a = 0.8;
  //ROS_INFO( " 8 init_person_companion_markers() ");
  //planning marker. Calculated robot trajectories in space x time coordinates
  personCompanion_planning_marker_.header.frame_id = this->fixed_frame;
  personCompanion_planning_marker_.ns = "p3";
  personCompanion_planning_marker_.type = visualization_msgs::Marker::LINE_LIST;
  personCompanion_planning_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_planning_marker_.lifetime = ros::Duration(1.3f);
  personCompanion_planning_marker_.id = 0;
  personCompanion_planning_marker_.scale.x = 0.05;
  personCompanion_planning_marker_.color.a = 0.3;
  personCompanion_planning_marker_.color.r = 0.1;
  personCompanion_planning_marker_.color.g = 0.2;
  personCompanion_planning_marker_.color.b = 0.9;
  // ROS_INFO( " 8.1 init_person_companion_markers(), =%d, horitzon_time=%f; ", scene_.get_akp_for_person_companion()->get_number_of_vertex(  ), scene_.get_akp_for_person_companion()->get_horizon_time());
  //unsigned int num_vertex=scene_.get_akp_for_person_companion()->get_number_of_vertex(  );

  //personCompanion_planning_marker_.points.reserve( num_vertex ); // a pelo! mirar de arreglar, pq peta sin sentido...

  //ROS_INFO( " 9 init_person_companion_markers() ");
  //cylinder marker, planning random goals
  personCompanion_planning_goals_marker_.header.frame_id = this->fixed_frame;
  personCompanion_planning_goals_marker_.ns = "goals";
  personCompanion_planning_goals_marker_.type = visualization_msgs::Marker::CYLINDER;
  personCompanion_planning_goals_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_planning_goals_marker_.lifetime = ros::Duration(1.3f);
  personCompanion_planning_goals_marker_.color.a = 0.6;
  personCompanion_planning_goals_marker_.color.r = 0.9;
  personCompanion_planning_goals_marker_.color.g = 0.1;
  personCompanion_planning_goals_marker_.color.b = 0.0;
  personCompanion_planning_goals_marker_.scale.x = 0.2;
  personCompanion_planning_goals_marker_.scale.y = 0.2;
  personCompanion_planning_goals_marker_.scale.z = 0.1;
  personCompanion_planning_goals_marker_.pose.position.z = 0.05;
  personCompanion_planning_goals_marker_.pose.orientation.w = 1.0;

	// ROS_INFO( " 10 init_person_companion_markers() ");
  //best path marker
  personCompanion_best_path_marker_.header.frame_id = this->fixed_frame;
  personCompanion_best_path_marker_.ns = "best3";
  personCompanion_best_path_marker_.type = visualization_msgs::Marker::LINE_STRIP;
  personCompanion_best_path_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_best_path_marker_.lifetime = ros::Duration(1.3f);
  personCompanion_best_path_marker_.scale.x = 0.05;
  personCompanion_best_path_marker_.color.a = 0.7;
  personCompanion_best_path_marker_.color.r = 0.9;
  personCompanion_best_path_marker_.color.g = 0.2;
  personCompanion_best_path_marker_.color.b = 0.2;
	
  //best path marker
  personCompanion_best_path2d_marker_ = personCompanion_best_path_marker_;
  personCompanion_best_path2d_marker_.ns = "best2";
  personCompanion_best_path2d_marker_.id = 0;
  //ROS_INFO( " 11 init_person_companion_markers() ");
  //non-dominated marker
  personCompanion_nd_path_marker_.header.frame_id = this->fixed_frame;
  personCompanion_nd_path_marker_.ns = "nd3";
  personCompanion_nd_path_marker_.type = visualization_msgs::Marker::LINE_LIST;
  personCompanion_nd_path_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_nd_path_marker_.lifetime = ros::Duration(1.3f);
  personCompanion_nd_path_marker_.id = 0;
  personCompanion_nd_path_marker_.scale.x = 0.05;
  personCompanion_nd_path_marker_.color.a = 0.7;
  personCompanion_nd_path_marker_.color.r = 1.0;
  personCompanion_nd_path_marker_.color.g = 0.5;
  personCompanion_nd_path_marker_.color.b = 0.0;
	
  //non dominated path marker
  personCompanion_nd_path2d_marker_ = personCompanion_nd_path_marker_;
  personCompanion_nd_path2d_marker_.ns = "nd2";
  personCompanion_nd_path2d_marker_.id = 0;
  //ROS_INFO( " 12 init_person_companion_markers() ");
  //laser obstacle marker
  personCompanion_laser_obstacle_marker_.header.frame_id = this->fixed_frame;
  personCompanion_laser_obstacle_marker_.ns = "obstacles";
  personCompanion_laser_obstacle_marker_.type = visualization_msgs::Marker::CYLINDER;
  personCompanion_laser_obstacle_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_laser_obstacle_marker_.lifetime = ros::Duration(1.3f);
  personCompanion_laser_obstacle_marker_.color.a = 0.7;
  personCompanion_laser_obstacle_marker_.color.r = 0.0;
  personCompanion_laser_obstacle_marker_.color.g = 0.0;
  personCompanion_laser_obstacle_marker_.color.b = 0.0;
  personCompanion_laser_obstacle_marker_.scale.x = 0.5;
  personCompanion_laser_obstacle_marker_.scale.y = 0.5;
  personCompanion_laser_obstacle_marker_.scale.z = 0.2;
  personCompanion_laser_obstacle_marker_.pose.position.z = 0.1;
  personCompanion_laser_obstacle_marker_.pose.orientation.w = 1.0;
  //ROS_INFO( " 13 init_person_companion_markers() ");
  //force marker, a summation of all forces (red)
  personCompanion_force_marker_.header.frame_id = this->fixed_frame;
  personCompanion_force_marker_.ns =  "forces_result";
  personCompanion_force_marker_.type = visualization_msgs::Marker::ARROW;
  personCompanion_force_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_force_marker_.lifetime = ros::Duration(1.3f);
  personCompanion_force_marker_.scale.x = 0.2;
  personCompanion_force_marker_.scale.y = 0.25;
  //personCompanion_force_marker_.scale.z = 0.2;
  personCompanion_force_marker_.color.a = 0.8;
  personCompanion_force_marker_.color.r = 1.0;
  personCompanion_force_marker_.color.g = 0.0;
  personCompanion_force_marker_.color.b = 0.0;
  personCompanion_force_marker_.points.push_back(  geometry_msgs::Point() );
  personCompanion_force_marker_.points.push_back(  geometry_msgs::Point() );

  //ROS_INFO( " 14 init_person_companion_markers() ");
  // marker to goal (blue)
  personCompanion_force_goal_marker_ = personCompanion_force_marker_;
  personCompanion_force_goal_marker_.ns =  "force_goal";
  personCompanion_force_goal_marker_.color.r = 0.0;
  personCompanion_force_goal_marker_.color.g = 0.4;
  personCompanion_force_goal_marker_.color.b = 1.0;

  // marker interaction with persons (green)
  personCompanion_force_int_person_marker_= personCompanion_force_marker_;
  personCompanion_force_int_person_marker_.color.r = 0.2;
  personCompanion_force_int_person_marker_.ns =  "force_to_pers";
  personCompanion_force_int_person_marker_.color.g = 0.85;
  personCompanion_force_int_person_marker_.color.b = 0.2;
  //ROS_INFO( " 15 init_person_companion_markers() ");
  //marker due to obstacles in the scene (black)
  personCompanion_force_obstacle_marker_ = personCompanion_force_marker_;
  personCompanion_force_obstacle_marker_.ns =  "force_to_obst";
  personCompanion_force_obstacle_marker_.color.r = 0.0;
  personCompanion_force_obstacle_marker_.color.g = 0.0;
  personCompanion_force_obstacle_marker_.color.b = 0.0;

  // force of interaction with robot (purple)
  personCompanion_force_int_robot_marker_ = personCompanion_force_marker_;
  personCompanion_force_int_robot_marker_.ns =  "force_to_robot";
  personCompanion_force_int_robot_marker_.color.r = 0.26;
  personCompanion_force_int_robot_marker_.color.g = 0.0;
  personCompanion_force_int_robot_marker_.color.b = 0.66;

  // marker force companion (cian)
  //force_companion_marker_ = personCompanion_force_marker_;
  //force_companion_marker_.color.r = 0.51;
  //force_companion_marker_.color.g = 0.243;
  //force_companion_marker_.color.b = 0.255;

  // ROS_INFO( " 16 init_person_companion_markers() ");
  //text marker
  personCompanion_text_marker_.header.frame_id = this->fixed_frame;
  personCompanion_text_marker_.ns = "text";
  personCompanion_text_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  personCompanion_text_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_text_marker_.lifetime = ros::Duration(1.3f);
  personCompanion_text_marker_.scale.z = 0.3;
  personCompanion_text_marker_.pose.position.z = 0.0;
  personCompanion_text_marker_.color.a = 1.0;
  //ROS_INFO( " 17 init_person_companion_markers() ");

  next_goal_marker_.header.frame_id = this->fixed_frame;
  next_goal_marker_.ns = "next_goal_Akp";
  next_goal_marker_.type = visualization_msgs::Marker::CYLINDER;
  next_goal_marker_.action = visualization_msgs::Marker::ADD;
  next_goal_marker_.lifetime = ros::Duration(1.3f);
  next_goal_marker_.color.a = 1.0;
  next_goal_marker_.color.r = 1.0;
  next_goal_marker_.color.g = 0.0;
  next_goal_marker_.color.b = 0.0;
  next_goal_marker_.scale.x = 1.4;
  next_goal_marker_.scale.y = 1.4;
  next_goal_marker_.scale.z = 1.0;
  next_goal_marker_.pose.position.z = 0.1;
  next_goal_marker_.pose.orientation.w = 1.0;

   goal_path_marker_.header.frame_id = this->fixed_frame;
   goal_path_marker_.ns = "goalPath";
   goal_path_marker_.type = visualization_msgs::Marker::SPHERE;
   goal_path_marker_.action = visualization_msgs::Marker::ADD;
   goal_path_marker_.lifetime = ros::Duration(1.3f);
   goal_path_marker_.color.a = 0.0;
   goal_path_marker_.color.r = 1.0;
   goal_path_marker_.color.g = 0.0;
   goal_path_marker_.color.b = 0.0;
   goal_path_marker_.scale.x = 1.3;
   goal_path_marker_.scale.y = 1.3;
   goal_path_marker_.scale.z = 1.3;
   goal_path_marker_.pose.position.z = 0.0;
   goal_path_marker_.pose.orientation.w = 1.0;

   goal_companion_marker_.header.frame_id = this->fixed_frame;
   goal_companion_marker_.ns = "goalCompanion";
   goal_companion_marker_.type = visualization_msgs::Marker::SPHERE;
   goal_companion_marker_.action = visualization_msgs::Marker::ADD;
   goal_companion_marker_.lifetime = ros::Duration(1.3f);
   goal_companion_marker_.color.a = 0.0;
   goal_companion_marker_.color.r = 0.0;
   goal_companion_marker_.color.g = 1.0;
   goal_companion_marker_.color.b = 0.0;
   goal_companion_marker_.scale.x = 1.3;
   goal_companion_marker_.scale.y = 1.3;
   goal_companion_marker_.scale.z = 1.3;
   goal_companion_marker_.pose.position.z = 0.0;
   goal_companion_marker_.pose.orientation.w = 1.0;
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

  //if((cov(0, 0)>1)||(cov(0, 1)>1)||(cov(1, 0)>1)||(cov(1, 1)>1)){
   // ROS_INFO(" cov(0, 0) = %f, cov(0, 1) = %f,cov(1, 0) = %f, cov(1, 1) = %f", cov(0, 0),cov(0, 1), cov(1, 0), cov(1, 1));
  // }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);
  //eig.compute(cov);
  marker.scale.x = sqrt( eig.eigenvalues()[0] ) + 0.001;//major vap
  marker.scale.y = sqrt( eig.eigenvalues()[1] ) + 0.001;//minor vap

  //if((marker.scale.x>1)||(marker.scale.y>1)){
   // ROS_INFO(" marker.scale.x = sqrt( eig.eigenvalues()[0] ) + 0.001= %f", marker.scale.x);
   // ROS_INFO(" marker.scale.y = sqrt( eig.eigenvalues()[1] ) + 0.001= %f", marker.scale.y);
  //}
  double angle = atan2( eig.eigenvectors()(1,0), eig.eigenvectors()(0,0) );
  marker.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
  
}

void AkpLocalPlanner::fill_forces_markers() // ok, creo...
{
  const std::list<Cperson_abstract *>* person_list = scene_.get_akp_for_person_companion()->get_scene( );
  Sforce force_to_goal, force_int_person , force_int_robot, force_obstacle, force_total, force_companion;
  geometry_msgs::Point ros_point, ros_point_ini;
  unsigned int cont_f = 0;


    //scaled person interaction force (green)
    personCompanion_force_int_person_marker_.pose.position.x=robot_pose_.x;
    personCompanion_force_int_person_marker_.pose.position.y=robot_pose_.y;
    personCompanion_force_int_person_marker_.pose.position.z=0.1;
    personCompanion_force_int_person_marker_.points[0]= ros_point_ini;
    Sforce force_to_tobot_with_person_companion=scene_.get_akp_for_person_companion()->get_force_int_between_person_comp_and_robot();
    ros_point.x = ros_point_ini.x + force_to_tobot_with_person_companion.fx;
    ros_point.y = ros_point_ini.y + force_to_tobot_with_person_companion.fy;
    personCompanion_force_int_person_marker_.points[1] = ros_point;
    personCompanion_force_int_person_marker_.id = cont_f;
    ++cont_f;
    //ROS_INFO(" force_to_tobot_with_person_companion.fx= %f; fy=%f", force_to_tobot_with_person_companion.fx,force_to_tobot_with_person_companion.fy);
    this->personCompanion_MarkerArray_msg_.markers.push_back(  personCompanion_force_int_person_marker_  );


  for( std::list<Cperson_abstract*>::const_iterator iit = person_list->begin();
          iit!=person_list->end(); iit++ )
  {
    // fill FORCES for each person
    //Sforce get_forces_person( Sforce&  force_to_goal, Sforce& force_int_person , Sforce& force_int_robot, Sforce& force_obstacle );
    force_total = (*iit)->get_forces_person( force_to_goal, force_int_person , force_int_robot, force_obstacle );
    ros_point_ini.x = (*iit)->get_current_pointV().x;
    ros_point_ini.y = (*iit)->get_current_pointV().y;



    //scaled person interaction force (green)
    personCompanion_force_int_person_marker_.pose.position.x=ros_point_ini.x;
    personCompanion_force_int_person_marker_.pose.position.y=ros_point_ini.y;
    personCompanion_force_int_person_marker_.pose.position.z=0.1;
    personCompanion_force_int_person_marker_.points[0]= ros_point_ini;
    ros_point.x = ros_point_ini.x + force_int_person.fx;
    ros_point.y = ros_point_ini.y + force_int_person.fy;
    personCompanion_force_int_person_marker_.points[1] = ros_point;
    personCompanion_force_int_person_marker_.id = cont_f;
    ++cont_f;
    this->personCompanion_MarkerArray_msg_.markers.push_back(  personCompanion_force_int_person_marker_  );

    // robot to person interaction force (pink)
    personCompanion_force_int_robot_marker_.pose.position.x=ros_point_ini.x;
    personCompanion_force_int_robot_marker_.pose.position.y=ros_point_ini.y;
    personCompanion_force_int_robot_marker_.pose.position.z=0.1;
    personCompanion_force_int_robot_marker_.points[0]= ros_point_ini;
    ros_point.x = ros_point_ini.x + force_int_robot.fx;
    ros_point.y = ros_point_ini.y + force_int_robot.fy;
    personCompanion_force_int_robot_marker_.points[1] = ros_point;
    personCompanion_force_int_robot_marker_.id = cont_f;
    ++cont_f;
    this->personCompanion_MarkerArray_msg_.markers.push_back(  personCompanion_force_int_robot_marker_  );

    //map obstacles interaction force (black)
    personCompanion_force_obstacle_marker_.pose.position.x=ros_point_ini.x;
    personCompanion_force_obstacle_marker_.pose.position.y=ros_point_ini.y;
    personCompanion_force_obstacle_marker_.pose.position.z=0.1;
    personCompanion_force_obstacle_marker_.points[0] = ros_point_ini;
    ros_point.x = ros_point_ini.x + force_obstacle.fx;
    ros_point.y = ros_point_ini.y + force_obstacle.fy;
    personCompanion_force_obstacle_marker_.points[1] =  ros_point;
    personCompanion_force_obstacle_marker_.id = cont_f;
    ++cont_f;
    this->personCompanion_MarkerArray_msg_.markers.push_back(  personCompanion_force_obstacle_marker_  );

  }
  //print robot forces
 
  force_total = scene_.get_akp_for_person_companion()->get_person_companion_akp()->get_forces_person_companion(force_to_goal, force_int_person , force_int_robot, force_obstacle, force_companion);
//force_total = scene_.get_akp_for_person_companion()->get_robot2()->get_forces_person_companion(force_to_goal, force_int_person , force_int_robot, force_obstacle, force_companion);

//ROS_INFO(" force_int_person.fx= %f; fy=%f", force_int_person.fx,force_int_person.fy);

  ros_point_ini.x = robot_pose_.x;//scene_.get_akp_for_person_companion()->get_person_companion_akp()->get_current_pointV().x;
  ros_point_ini.y = robot_pose_.y;//scene_.get_akp_for_person_companion()->get_person_companion_akp()->get_current_pointV().y;

  double mult=2;

  if(debug_forces_akp_person_companion_){
    ROS_INFO("AkpLocalPlanner::markers, ros_point_ini.x=%f=",ros_point_ini.x);
    ROS_INFO("AkpLocalPlanner::markers, ros_point_ini.y=%f=",ros_point_ini.y);

    ROS_INFO("(ROS) AkpLocalPlanner::markers, force_to_goal.fx=%f=",force_to_goal.fx);
    ROS_INFO("(ROS) AkpLocalPlanner::markers, force_to_goal.fy=%f=",force_to_goal.fy);

    ROS_INFO("(ROS) AkpLocalPlanner::markers, force_int_person.fx=%f",force_int_person.fx);
    ROS_INFO("(ROS) AkpLocalPlanner::markers, force_int_person.fy=%f",force_int_person.fy);

    ROS_INFO("(ROS) AkpLocalPlanner::markers, force_int_robot.fx=%f=",force_int_robot.fx);
    ROS_INFO("(ROS) AkpLocalPlanner::markers, force_int_robot.fy=%f=",force_int_robot.fy);

    ROS_INFO("(ROS) AkpLocalPlanner::markers, force_obstacle.fx=%f=",force_obstacle.fx);
    ROS_INFO("(ROS) AkpLocalPlanner::markers, force_obstacle.fy=%f=",force_obstacle.fy);

    ROS_INFO("(ROS) AkpLocalPlanner::markers, force_companion.fx=%f=",force_companion.fx);
    ROS_INFO("(ROS) AkpLocalPlanner::markers, force_companion.fy=%f=",force_companion.fy);

  }
  //scaled force to goal: ( blue )

  if(debug_forces_akp_person_companion_){
    ROS_INFO("(ROS) cont_f=%d", cont_f);    
  }

  if(this->person_companion_goal_force_marker)
  {
    if(debug_forces_akp_person_companion_){
      ROS_INFO("(ROS) robot force goal");    
      ROS_INFO("(ROS) force_to_goal.fx=%f",force_to_goal.fx);
      ROS_INFO("(ROS) force_to_goal.fy=%f",force_to_goal.fy);  
    }

    personCompanion_force_goal_marker_.points[0]= ros_point_ini;
    ros_point.x = ros_point_ini.x + mult*force_to_goal.fx;
    ros_point.y = ros_point_ini.y + mult*force_to_goal.fy;


   if(debug_forces_akp_person_companion_){
      ROS_INFO("(ROS) ros_point_ini.x=%f",ros_point_ini.x);
      ROS_INFO("(ROS) ros_point_ini.y=%f",ros_point_ini.y);
      ROS_INFO("personCompanion_force_goal_marker_ (ROS) ros_point.x=%f",ros_point.x);
      ROS_INFO("personCompanion_force_goal_marker_ (ROS) ros_point.y=%f",ros_point.y);
      ROS_INFO("personCompanion_force_goal_marker_ (ROS) inc_x=%f; inc_y=%f",ros_point_ini.x-ros_point.x,ros_point_ini.y-ros_point.y);
    }

   // personCompanion_force_goal_marker_.pose.position.x=ros_point_ini.x;
   // personCompanion_force_goal_marker_.pose.position.y=ros_point_ini.y;
   // personCompanion_force_goal_marker_.pose.position.z=0.1;

    personCompanion_force_goal_marker_.points[1] = ros_point;
    personCompanion_force_goal_marker_.id = cont_f;
    ++cont_f;
    this->personCompanion_MarkerArray_msg_.markers.push_back(  personCompanion_force_goal_marker_  );
  }

  if(debug_forces_akp_person_companion_){
    ROS_INFO("(ROS) cont_f=%d", cont_f);    
  }
  //scaled person interaction force (green)
  if(this->person_companion_person_forces_marker)
  { 
    if(debug_forces_akp_person_companion_){
      ROS_INFO("(ROS) robot force person");
      ROS_INFO("(ROS) force_int_person.fx=%f",force_int_person.fx);
      ROS_INFO("(ROS) force_int_person.fy=%f",force_int_person.fy);
    }
    //personCompanion_force_int_person_marker_.pose.position.x=ros_point_ini.x;
    //personCompanion_force_int_person_marker_.pose.position.y=ros_point_ini.y;
    //personCompanion_force_int_person_marker_.pose.position.z=0.1;

    personCompanion_force_int_person_marker_.points[0]= ros_point_ini;
    ros_point.x = ros_point_ini.x + mult*force_int_person.fx;
    ros_point.y = ros_point_ini.y + mult*force_int_person.fy;
    personCompanion_force_int_person_marker_.points[1] = ros_point;

    if(debug_forces_akp_person_companion_){
      ROS_INFO("(ROS) ros_point_ini.x=%f",ros_point_ini.x);
      ROS_INFO("(ROS) ros_point_ini.y=%f",ros_point_ini.y);
      ROS_INFO("personCompanion_force_int_person_marker_ (ROS) ros_point.x=%f",ros_point.x);
      ROS_INFO("personCompanion_force_int_person_marker_ (ROS) ros_point.y=%f",ros_point.y);
      ROS_INFO("personCompanion_force_int_person_marker_ (ROS) inc_x=%f; inc_y=%f",ros_point_ini.x-ros_point.x,ros_point_ini.y-ros_point.y);
    }

    personCompanion_force_int_person_marker_.id = cont_f;
    ++cont_f;
    this->personCompanion_MarkerArray_msg_.markers.push_back(  personCompanion_force_int_person_marker_  );
  }

  if(debug_forces_akp_person_companion_){
    ROS_INFO("(ROS) cont_f=%d", cont_f); 
  }   
  //map obstacles interaction force (black)
  if(this->person_companion_obstacles_forces_marker)
  {
    if(debug_forces_akp_person_companion_){
      ROS_INFO("(ROS) robot force obstacles");
    }

    //personCompanion_force_obstacle_marker_.pose.position.x=ros_point_ini.x;
    //personCompanion_force_obstacle_marker_.pose.position.y=ros_point_ini.y;
   // personCompanion_force_obstacle_marker_.pose.position.z=0.1;

    personCompanion_force_obstacle_marker_.points[0] = ros_point_ini;
    ros_point.x = ros_point_ini.x + mult*force_obstacle.fx;
    ros_point.y = ros_point_ini.y + mult*force_obstacle.fy;
    personCompanion_force_obstacle_marker_.points[1] =  ros_point;
    personCompanion_force_obstacle_marker_.id = cont_f;
    ++cont_f;

    if(debug_forces_akp_person_companion_){
      ROS_INFO("(ROS) ros_point_ini.x=%f",ros_point_ini.x);
      ROS_INFO("(ROS) ros_point_ini.y=%f",ros_point_ini.y);
      ROS_INFO("personCompanion_force_obstacle_marker_ (ROS) ros_point.x=%f",ros_point.x);
      ROS_INFO("personCompanion_force_obstacle_marker_ (ROS) ros_point.y=%f",ros_point.y);
      ROS_INFO("personCompanion_force_obstacle_marker_ (ROS) inc_x=%f; inc_y=%f",ros_point_ini.x-ros_point.x,ros_point_ini.y-ros_point.y);
    }

    this->personCompanion_MarkerArray_msg_.markers.push_back(  personCompanion_force_obstacle_marker_  );
  }

  if(debug_forces_akp_person_companion_){
    ROS_INFO("(ROS) cont_f=%d", cont_f);
  }   
  //weighted resultant force (red)
  if(this->person_companion_resultant_force_marker)
  {
    if(debug_forces_akp_person_companion_){
      ROS_INFO("(ROS) robot force resultan");
    }

    personCompanion_force_marker_.pose.position.x=ros_point_ini.x;
    personCompanion_force_marker_.pose.position.y=ros_point_ini.y;
    personCompanion_force_marker_.pose.position.z=0.1;
    personCompanion_force_marker_.points[0] =ros_point_ini;
    ros_point.x = ros_point_ini.x + mult*force_total.fx;
    ros_point.y = ros_point_ini.y + mult*force_total.fy;
    personCompanion_force_marker_.points[1] = ros_point;
    personCompanion_force_marker_.id = cont_f;
    ++cont_f;

    if(debug_forces_akp_person_companion_){
      ROS_INFO("(ROS) ros_point_ini.x=%f",ros_point_ini.x);
      ROS_INFO("(ROS) ros_point_ini.y=%f",ros_point_ini.y);
      ROS_INFO("personCompanion_force_marker_ (ROS) ros_point.x=%f",ros_point.x);
      ROS_INFO("personCompanion_force_marker_ (ROS) ros_point.y=%f",ros_point.y);
      ROS_INFO("personCompanion_force_marker_ (ROS) inc_x=%f; inc_y=%f",ros_point_ini.x-ros_point.x,ros_point_ini.y-ros_point.y);
    }

    this->personCompanion_MarkerArray_msg_.markers.push_back(  personCompanion_force_marker_  );
  }

  if(debug_forces_akp_person_companion_){
    ROS_INFO("(ROS) cont_f=%d", cont_f);    
  }
  //scaled force to goal: ( Cian )
  //if(this->person_companion_companion_force_marker)
  // {
 /*   if(debug_antes_subgoals_entre_AKP_goals_){
      ROS_INFO("(ROS) robot force goal companion");
      ROS_INFO("(ROS) ini ros_point_ini.x=%f",ros_point_ini.x);
      ROS_INFO("(ROS) ini ros_point_ini.y=%f",ros_point_ini.y);
      ROS_INFO("(ROS) force_companion.fx=%f",force_companion.fx);
      ROS_INFO("(ROS) force_companion.fy=%f",force_companion.fy);
    }

    force_companion_marker_.points[0]= ros_point_ini;
    ros_point.x = ros_point_ini.x + 2*force_companion.fx;
    ros_point.y = ros_point_ini.y + 2*force_companion.fy;

   if(debug_real_test_companion_){
      ROS_INFO("force_companion_marker_ (ROS) final ros_point.x=%f",ros_point.x);
      ROS_INFO("force_companion_marker_ (ROS) final ros_point.y=%f",ros_point.y);
    }

    force_companion_marker_.points[1] = ros_point;
    force_companion_marker_.id = cont_f;
    ++cont_f;
    MarkerArray_msg_.markers.push_back(  force_companion_marker_  );
  //}

  // const Crobot* act_robot=scene_.get_akp_for_person_companion()->get_robot();
  Sdestination robot_companion_dest= scene_.get_akp_for_person_companion()->get_person_companion_akp()->get_companion_force_goal();
  //scene_.get_akp_for_person_companion()->get_robot()->get_companion_force_goal(robot_companion_dest);
  force_companion_goal_marker_.pose.position.x = robot_companion_dest.x;
  force_companion_goal_marker_.pose.position.y = robot_companion_dest.y;
  force_companion_goal_marker_.id = 20;
  // MarkerArray_msg_.markers.push_back( force_companion_goal_marker_ );*/
  
}

void AkpLocalPlanner::fill_scene_markers() //ok, creo...
{

  unsigned int cont(0);
  const std::vector<Sdestination>* dest = scene_.get_akp_for_person_companion()->get_destinations();
  for( unsigned int i = 0; i < dest->size(); ++i)
  {
    personCompanion_cylinder_marker_.pose.position.x = dest->at(i).x;
    personCompanion_cylinder_marker_.pose.position.y = dest->at(i).y;
    personCompanion_cylinder_marker_.id = cont;
    this->personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_cylinder_marker_ );
    ++cont;
  } 

  //draw robot destination, both local and global
  personCompanion_robot_goal_marker_.pose.position.x = scene_.get_akp_for_person_companion()->get_robot_local_goal().x;
  personCompanion_robot_goal_marker_.pose.position.y = scene_.get_akp_for_person_companion()->get_robot_local_goal().y;
  personCompanion_robot_goal_marker_.id = cont;
  ++cont;
  this->personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_robot_goal_marker_ );
  personCompanion_robot_goal_marker_.pose.position.x = scene_.get_akp_for_person_companion()->get_robot_goal().x;
  personCompanion_robot_goal_marker_.pose.position.y = scene_.get_akp_for_person_companion()->get_robot_goal().y;
  personCompanion_robot_goal_marker_.id = cont;
  ++cont;
  this->personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_robot_goal_marker_ );
  
  //if slicing mode, plot the set of destinations
  if( goal_providing_mode_ == AkpLocalPlanner::Slicing_global )
  {
    for( unsigned int i = 0; i< sliced_global_plan_.size(); ++i )
    {
      personCompanion_robot_subgoal_marker_.pose.position.x = sliced_global_plan_[i].pose.position.x;
      personCompanion_robot_subgoal_marker_.pose.position.y = sliced_global_plan_[i].pose.position.y;
      personCompanion_robot_subgoal_marker_.id = cont;
      ++cont;
      this->personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_robot_subgoal_marker_ );
    }
  }

  //draw workspace of the robot
  double r = scene_.get_akp_for_person_companion()->get_workspace_radii();
	geometry_msgs::Point ros_point, center_point;
  personCompanion_workspace_marker_.points.clear();
  personCompanion_workspace_marker_.id = cont;
  ++cont;
  const std::vector<Spose>* plans = scene_.get_akp_for_person_companion()->get_person_companion_akp()->get_robot_planning_trajectory();

  if(!plans->empty()){
    center_point.x = plans->at(0).x;
    center_point.y = plans->at(0).y;
  }else{
    center_point.x = robot_pose_.x;
    center_point.y = robot_pose_.y;
  }


  for( unsigned int i = 0; i < 40; ++i )
  {

  /*
  const std::vector<Spose>* plans = scene_.get_akp_for_person_companion()->get_robot()->get_robot_planning_trajectory();
  const std::vector<Sedge_tree>* edges = scene_.get_akp_for_person_companion()->get_plan_edges();
	geometry_msgs::Point ros_point;
  double ini_time = plans->front().time_stamp;
	
	// plot 3d plan, complete tree
  personCompanion_planning_marker_.points.clear();
  for( unsigned int i = 1; i< plans->size(); ++i )
  {
    //lines segments beetwen 2 points
    ros_point.x = plans->at(i).x;
    ros_point.y = plans->at(i).y;
    ros_point.z = plans->at(i).time_stamp - ini_time;
*/
    ros_point.x = center_point.x + r*cos( 2*i*PI/40 );
    ros_point.y = center_point.y + r*sin( 2*i*PI/40 );
    //ros_point.x = robot_pose_.x + r*cos( 2*i*PI/40 );
    //ros_point.y = robot_pose_.y + r*sin( 2*i*PI/40 );
    personCompanion_workspace_marker_.points.push_back( ros_point ); 
    ros_point.x = center_point.x + r*cos( (2*i+1)*PI/40 );
    ros_point.y = center_point.y + r*sin( (2*i+1)*PI/40 );
    //ros_point.x = robot_pose_.x + r*cos( 2*i*PI/40 );
    //ros_point.y = robot_pose_.y + r*sin( 2*i*PI/40 );
    personCompanion_workspace_marker_.points.push_back( ros_point ); 
  }
  this->personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_workspace_marker_ );

  // robot_goal_marker!!!
  Spoint robot_goal=scene_.get_akp_for_person_companion()->get_robot_goal();
  personCompanion_robot_goal_marker2_.pose.position.x = robot_goal.x;
  personCompanion_robot_goal_marker2_.pose.position.y = robot_goal.y;
  personCompanion_robot_goal_marker2_.id = cont;
  ++cont;
  this->personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_robot_goal_marker2_ );

  Spoint robot_goal2=scene_.get_akp_for_person_companion()->get_external_robot_goal();
  personCompanion_robot_goal_marker3_.pose.position.x = robot_goal2.x;
  personCompanion_robot_goal_marker3_.pose.position.y = robot_goal2.y;
  personCompanion_robot_goal_marker3_.id = cont;
  ++cont;
  this->personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_robot_goal_marker3_ );

}

void AkpLocalPlanner::fill_planning_markers_2d()
{
  const std::vector<Spose>* plans = scene_.get_akp_for_person_companion()->get_person_companion_akp()->get_robot_planning_trajectory();
	geometry_msgs::Point ros_point;
  //random goals printing -------------------------------------------------------------------
  const std::vector<Spoint>* goals =  scene_.get_akp_for_person_companion()->get_random_goals();
  for( unsigned int i = 0; i< goals->size(); ++i )
  {
    personCompanion_planning_goals_marker_.pose.position.x = goals->at(i).x;
    personCompanion_planning_goals_marker_.pose.position.y = goals->at(i).y;
    personCompanion_planning_goals_marker_.id = i;
    this->personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_planning_goals_marker_ );
  }
  
  // plot non-dominated solutions in 2d ..................-------------------------------------
  const std::vector<unsigned int>* nondominated_path_index = scene_.get_akp_for_person_companion()->get_robot_nondominated_plan_index();
  const std::vector<unsigned int>* nondominated_end_of_path_index = scene_.get_akp_for_person_companion()->get_robot_nondominated_end_of_plan_index();
  personCompanion_nd_path2d_marker_.points.clear();
  //ROS_INFO("size = %d" , nondominated_path_index->size());
  for( unsigned int i = 0; i< nondominated_path_index->size(); i++ )
  {
    ros_point.x = plans->at(nondominated_path_index->at(i)).x;
    ros_point.y = plans->at(nondominated_path_index->at(i)).y;
    ros_point.z = 0.0;
    personCompanion_nd_path2d_marker_.points.push_back( ros_point );
  }
  //paths always even
  this->personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_nd_path2d_marker_ );
  
  // plot text id of paths, if MultiObjective function mode is activated ---------------------
  personCompanion_text_marker_.id = 0;
  unsigned int i = 0;
  for(  ; i< nondominated_end_of_path_index->size(); i++ )
  {
    personCompanion_text_marker_.id++; 
    personCompanion_text_marker_.pose.position.x = plans->at(nondominated_end_of_path_index->at(i)).x;
    personCompanion_text_marker_.pose.position.y = plans->at(nondominated_end_of_path_index->at(i)).y;
    std::stringstream idText;
    idText << nondominated_end_of_path_index->at(i);
    personCompanion_text_marker_.text = idText.str();
    this->personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_text_marker_ );
  }
  unsigned int text_markers_size = i;
	personCompanion_text_marker_.action = visualization_msgs::Marker::DELETE;
  for( ; i< text_markers_old_size_ ; i++)
  {
    personCompanion_text_marker_.id++;
    this->personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_text_marker_ );
  }
	personCompanion_text_marker_.action = visualization_msgs::Marker::ADD;
  text_markers_old_size_ = text_markers_size;

}

void AkpLocalPlanner::fill_planning_markers_3d() //ok, creo...
{
  const std::vector<Spose>* plans = scene_.get_akp_for_person_companion()->get_person_companion_akp()->get_robot_planning_trajectory();
  const std::vector<Sedge_tree>* edges = scene_.get_akp_for_person_companion()->get_plan_edges();
  geometry_msgs::Point ros_point;
  double ini_time = plans->front().time_stamp;
	
  // plot 3d plan, complete tree
  personCompanion_planning_marker_.points.clear();
  for( unsigned int i = 1; i< plans->size(); ++i )
  {
    //lines segments beetwen 2 points
    ros_point.x = plans->at(i).x;
    ros_point.y = plans->at(i).y;
    ros_point.z = plans->at(i).time_stamp - ini_time;
    personCompanion_planning_marker_.points.push_back( ros_point );
    //ROS_INFO("message of size %d, at %d and parent %d", plans->size(), i, edges->at(i).parent);
    ros_point.x = plans->at( edges->at(i).parent ).x;
    ros_point.y = plans->at( edges->at(i).parent ).y;
    ros_point.z = plans->at(edges->at(i).parent).time_stamp - ini_time;
    personCompanion_planning_marker_.points.push_back( ros_point );
  }
  this->personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_planning_marker_ ); 
    
  //plot best trajectory path in 3d
  const std::vector<unsigned int>* best_path_index = scene_.get_akp_for_person_companion()->get_robot_plan_index();
  personCompanion_best_path_marker_.points.clear();
  //ROS_INFO("size = %d" , personCompanion_best_path_marker_.points.size());
  for( unsigned int i = 0; i< best_path_index->size(); i++ )
  {
    ros_point.x = plans->at(best_path_index->at(i)).x;
    ros_point.y = plans->at(best_path_index->at(i)).y;
    ros_point.z = plans->at(best_path_index->at(i)).time_stamp - ini_time;
    personCompanion_best_path_marker_.points.push_back( ros_point );
  }
  this->personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_best_path_marker_ );
  
  // plot non-dominated solutions in 3d
  const std::vector<unsigned int>* nondominated_path_index = scene_.get_akp_for_person_companion()->get_robot_nondominated_plan_index();
  personCompanion_nd_path_marker_.points.clear();
  //ROS_INFO("size = %d" , nondominated_path_index->size());
  for( unsigned int i = 0; i< nondominated_path_index->size(); i++ )
  {
    ros_point.x = plans->at(nondominated_path_index->at(i)).x;
    ros_point.y = plans->at(nondominated_path_index->at(i)).y;
    ros_point.z = plans->at(nondominated_path_index->at(i)).time_stamp - ini_time;
    personCompanion_nd_path_marker_.points.push_back( ros_point );
  }
  //paths always even
  this->personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_nd_path_marker_ );

}


void AkpLocalPlanner::fill_people_prediction_markers_2d()  // ok, creo...
{
  //plot people trajectories 2d
  const std::vector<unsigned int>* best_path_index = scene_.get_akp_for_person_companion()->get_robot_plan_index();
  const std::list<Cperson_abstract *>* person_list = scene_.get_akp_for_person_companion()->get_scene( );
  const std::vector<SpointV_cov>* traj;
  unsigned int cont_2d = 0;
  for( std::list<Cperson_abstract*>::const_iterator iit = person_list->begin();
        iit!=person_list->end(); iit++ )
  {
    traj = (*iit)->get_planning_trajectory();
    unsigned int id_person=(*iit)->get_id();
    //ROS_INFO("id_person= %d", id_person);


    //if((id_person==1000)||(id_person==1015)||(id_person==1019)||(id_person==774)){ //||(id_person==733)
      if ( traj->size() > best_path_index->size() )
      //if ( traj->empty() )
      {

        unsigned int less_prediction= (best_path_index->size())/2;

          //ROS_INFO("traj->size()= %d", traj->size());
          //ROS_INFO("best_path_index->size()= %d", best_path_index->size());
          //ROS_INFO("less_prediction= %d", less_prediction);
        for( unsigned int i = 0; i< best_path_index->size(); ++i , ++cont_2d)
       // for( unsigned int i = 0; i< best_path_index->size()-less_prediction; ++i , ++cont_2d)
        {
          fill_my_covariance_marker( personCompanion_pred_traj2d_marker_,  traj->at(best_path_index->at(i)) );
          personCompanion_pred_traj2d_marker_.id = cont_2d;
          this->personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_pred_traj2d_marker_ );
        }
      }
    //}

  }

}



void AkpLocalPlanner::fill_people_prediction_markers_3d() // ok, creo...
{

  const std::list<Cperson_abstract *>* person_list = scene_.get_akp_for_person_companion()->get_scene( );
  const std::vector<SpointV_cov>* traj;
  unsigned int cont = 0;
  double time_stamp_ini = scene_.get_akp_for_person_companion()->get_time();


  //ROS_INFO( "person list %d" , person_list->size() );
  for( std::list<Cperson_abstract*>::const_iterator iit = person_list->begin();
          iit!=person_list->end(); iit++ )
  {
    if( (*iit)->get_planning_trajectory()->size() <= 1 )
      traj = (*iit)->get_prediction_trajectory();
    else
      traj = (*iit)->get_planning_trajectory();

    unsigned int less_prediction= 13;//= (traj->size())/2;

   // ROS_INFO("traj->size()= %d", traj->size());
    //ROS_INFO("less_prediction= %d", less_prediction);

    unsigned int id_person=(*iit)->get_id();
    //ROS_INFO("id_person= %d", id_person);
    //if((id_person==476)||(id_person==773)||(id_person==775)||(id_person==774)){ //||(id_person==733)
      //fill prediction covariances
      for( unsigned int i = 0; i< traj->size(); ++i , ++cont)
      //for( unsigned int i = 0; i< less_prediction; ++i , ++cont)
      {
         //ROS_INFO("i= %d", i);
        fill_my_covariance_marker( personCompanion_pred_traj_marker_,  traj->at(i) );
        personCompanion_pred_traj_marker_.pose.position.z = traj->at(i).time_stamp - time_stamp_ini;//2d cov elevated the time azis(z)
        personCompanion_pred_traj_marker_.id = cont;
        this->personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_pred_traj_marker_ );
      }
    //}

  }

}


void AkpLocalPlanner::fill_laser_obstacles() // ok, creo...
{
  unsigned int cont(0);
  const std::vector<Spoint>* obstacles = scene_.get_akp_for_person_companion()->get_laser_obstacles();
  for( unsigned int i = 0; i < obstacles->size(); ++i)
  {
    personCompanion_laser_obstacle_marker_.pose.position.x = obstacles->at(i).x;
    personCompanion_laser_obstacle_marker_.pose.position.y = obstacles->at(i).y;
    personCompanion_laser_obstacle_marker_.id = cont;
    ++cont;
    this->personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_laser_obstacle_marker_ );
  }

}

void AkpLocalPlanner::fill_best_path_2d()  // ok
{
  geometry_msgs::Point ros_point;
  personCompanion_best_path2d_marker_.points.clear();
  const std::vector<Spose>* plans = scene_.get_akp_for_person_companion()->get_person_companion_akp()->get_robot_planning_trajectory();
  const std::vector<unsigned int>* best_path_index = scene_.get_akp_for_person_companion()->get_robot_plan_index(); //OK
  //const std::vector<Spose>* best_plan = scene_.get_akp_for_person_companion()->get_best_planning_trajectory();

  //ROS_ERROR("best path size = %d, and robot plan size = %d" , best_path_index->size(), plans->size()  );
  for( unsigned int i = 0; i< best_path_index->size(); i++ )
  {
    //ROS_INFO ( "size best_path = %d, and size of plans %d ", best_path_index->size(), plans->size()  );
    //ROS_INFO("filling %d, and best path index = %d" , i, best_path_index->at(i));
    ros_point.x = plans->at(best_path_index->at(i)).x;
    ros_point.y = plans->at(best_path_index->at(i)).y;
    ros_point.z = 0.0;
    personCompanion_best_path2d_marker_.points.push_back( ros_point );
  }

  this->personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_best_path2d_marker_ );
}


/*  [service callbacks] */
bool AkpLocalPlanner::resetCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) 
{ 
  ROS_INFO("AkpLocalPlanner::resetCallback: New Request Received!"); 

  //use appropiate mutex to shared variables if necessary 
  //this->reset_mutex_enter(); 

  if( simulation_mode_ == AkpLocalPlanner::Density_incremental )
  {
    //TODO: set the incremental proportion
    //n_persons_ = (num_experiment_/25 + 1)*25;
    //scene_.set_number_virtual_people( n_persons_ );
  }

  
  this->alg_.lock(); 
  ROS_INFO("AkpLocalPlanner::resetCallback: Processin New Request!"); 
  //do operations with req and output on res 
  scene_.clear_scene();
  scene_.set_number_virtual_people( n_persons_ );
  this->alg_.unlock(); 

  //unlock previously blocked shared variables 
  //this->reset_mutex_exit(); 

  return true; 
}


// subscriber call_back
/*
void AkpLocalPlanner::status_init_callback(const iri_perception_msgs::restartSim::ConstPtr& msg){
 ROS_INFO("[People_companion] AkpLocalPlanner::init_simulationsCallback: New Request Received!");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->status_init_mutex_enter(); //
   switch(msg->state){

			case 0:
          Action_ROS_=Cplan_local_nav::START;
      break;
      case 1:
          Action_ROS_=Cplan_local_nav::ITER;
      break;
  
    }

    this->scene_.set_state_Action_person_companion(Action_ROS_);

   // Action_ROS_=req.init.state;

    ROS_INFO("AkpLocalPlanner::status_init_callback: msg->state=%d",Action_ROS_);
    // hará falta incluir al inicio de la iteración el set state.    



     Spoint person_companion_init_point=Spoint(msg->initial_position[1].x,msg->initial_position[1].y,ros::Time::now().toSec());    //  SpointV_cov(double x_ , double y_, double time_stamp_, double vx_, double vy_,const std::vector<double>& cov_)


    this->scene_.set_actual_goal_to_return_person_comp(person_companion_init_point); // entras al nodo el init person companion.


  ROS_INFO("AkpLocalPlanner::status_init_callback: Init person companion Spoint; orientation=%f",msg->orientation[1]);
     person_companion_init_point.print();
    

     Spoint person_goal_init_point=Spoint(msg->initial_position[2].x,msg->initial_position[2].y,ros::Time::now().toSec()); 


    this->scene_.set_person_goal_init_poin(person_goal_init_point);


 ROS_INFO("AkpLocalPlanner::status_init_callback: Init person goal Spoint; orientation=%f",msg->orientation[2]);
    person_goal_init_point.print();

  // + generar el mensaje a enviar al server.
  

  //ROS_INFO("NodoPruebaBacioAlgNode::init_simulationsCallback: Processing New Request!");
  //do operations with req and output on res
  //res.data2 = req.data1 + my_var;

  //unlock previously blocked shared variables
  //this->status_init_mutex_exit();
  //this->alg_.unlock();

}*/
/* [service callback] => restart persons to initial position */

bool AkpLocalPlanner::init_simulationsCallback(iri_perception_msgs::InitialiceSim::Request &req, iri_perception_msgs::InitialiceSim::Response &res)
{
  //ROS_INFO("[People_companion] AkpLocalPlanner::init_simulationsCallback: New Request Received!");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->init_simulations_mutex_enter(); //
   switch(req.init.state){

			case 0:
          Action_ROS_=Cplan_local_nav::START;
      break;
      case 1:
          Action_ROS_=Cplan_local_nav::ITER;
      break;
  
    }
    this->scene_.set_state_Action_person_companion(Action_ROS_);
    this->scene_.get_akp_for_person_companion()->set_state_Action(Action_ROS_);

   switch(req.init.act_case){

			case 0:
          ROS_INFO(" [PERSONS] AkpLocalPlanner::computeVelocityCommands: case0");
         Actual_case_ROS_=Cplan_local_nav::case0;
      break;
      case 1:
            ROS_INFO(" [PERSONS] AkpLocalPlanner::computeVelocityCommands: case1");
           Actual_case_ROS_=Cplan_local_nav::case1;
      break;
      case 2:
            ROS_INFO(" [PERSONS] AkpLocalPlanner::computeVelocityCommands: case2");
           Actual_case_ROS_=Cplan_local_nav::case2;
      break;
  
    }
    this->scene_.set_actual_simulation_case(Actual_case_ROS_);
    this->scene_.get_akp_for_person_companion()->set_actual_case(Actual_case_ROS_);

   // Action_ROS_=req.init.state;

   // ROS_INFO("AkpLocalPlanner::init_simulationsCallback: req.state=%d",Action_ROS_);
    // hará falta incluir al inicio de la iteración el set state.    


     Spoint person_companion_init_point1=Spoint(req.init.initial_position[0].x,req.init.initial_position[0].y,ros::Time::now().toSec()); 
     Spoint person_companion_init_point2=Spoint(req.init.initial_position[1].x,req.init.initial_position[1].y,ros::Time::now().toSec());    //  SpointV_cov(double x_ , double y_, double time_stamp_, double vx_, double vy_,const std::vector<double>& cov_)

    this->scene_.set_actual_goal_to_return_person_comp1(person_companion_init_point1);

    this->scene_.set_actual_goal_to_return_person_comp2(person_companion_init_point2); // entras al nodo el init person companion.


  //ROS_INFO("AkpLocalPlanner::init_simulationsCallback: Init person companion Spoint; orientation=%f",req.init.orientation[1]);
  //   person_companion_init_point1.print();
  //  person_companion_init_point2.print();

     Spoint person_goal_init_point=Spoint(req.init.initial_position[2].x,req.init.initial_position[2].y,ros::Time::now().toSec()); 


    this->scene_.set_person_goal_init_poin(person_goal_init_point);


 //ROS_INFO("AkpLocalPlanner::init_simulationsCallback: Init person goal Spoint; orientation=%f",req.init.orientation[2]);
 //   person_goal_init_point.print();

  // + generar el mensaje a enviar al server.
  

  //ROS_INFO("NodoPruebaBacioAlgNode::init_simulationsCallback: Processing New Request!");
  //do operations with req and output on res
  //res.data2 = req.data1 + my_var;

  //unlock previously blocked shared variables
  //this->init_simulations_mutex_exit();
  //this->alg_.unlock();

  return true;
}

void AkpLocalPlanner::init_simulations_mutex_enter(void)
{
  pthread_mutex_lock(&this->init_simulations_mutex_);
}

void AkpLocalPlanner::init_simulations_mutex_exit(void)
{
  pthread_mutex_unlock(&this->init_simulations_mutex_);
}


// to stop the code, to evaluate the costs


void AkpLocalPlanner::cmd_vel_stop_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  //if(debug_stop_node_to_evaluate_costs_){
    ROS_INFO("[simulated person] AkpLocalPlanner::cmd_vel_stop_callback: New Message Received");
  //}

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->cmd_vel_stop_mutex_enter();
  //std::cout << msg->data << std::endl;

  double linear_x=msg->linear.x;
  double linear_y=msg->linear.y;
  double linear_z=msg->linear.z;
  double angular_x=msg->angular.x;
  double angular_y=msg->angular.y;
  double angular_z=msg->angular.z;

  if(debug_stop_node_to_evaluate_costs_){
    std::cout <<"[simulated person] linear_x="<<linear_x<<std::endl;
    std::cout <<"[simulated person] linear_y="<<linear_y<<std::endl;
    std::cout <<"[simulated person] linear_z="<<linear_z<<std::endl;
    std::cout <<"[simulated person] angular_x="<<angular_x<<std::endl;
    std::cout <<"[simulated person] angular_y="<<angular_y<<std::endl;
    std::cout <<"[simulated person] angular_z="<<angular_z<<std::endl;
  }

  if((linear_x==7)&&(angular_z==7)){
     if(debug_stop_node_to_evaluate_costs_){
      ROS_INFO("[simulated person] NEW ITERATION");
      std::cout <<std::endl<<std::endl<<std::endl;
     }
      //this->start=true;
      this->current_state=HSQ_IT;
  }

  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->cmd_vel_stop_mutex_exit();
}

// for group simulation: other person, dabo tracks call back:

void AkpLocalPlanner::other_person_companion_tracks_callback(const iri_perception_msgs::detectionArray::ConstPtr& msg)
{
  //ROS_INFO("AkpLocalPlanner::other_person_companion_tracks_callback: New Message Received");
  //ROS_INFO("AkpLocalPlanner(other_person_companion)::tracks_callback: tracks are in %s frame instead of %s", msg->header.frame_id.c_str(), this->fixed_frame.c_str());
  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->dabo1_tracks_mutex_enter();
  this->fixed_frame = "/map";
// ROS_INFO("AkpLocalPlanner::tracks_callback: New Message Received"); 
  
  //ROS_INFO("AkpLocalPlanner::tracks_callback: msg->header.frame_id= %s",msg->header.frame_id.c_str());
  //ROS_INFO("AkpLocalPlanner::tracks_callback: this->fixed_frame= %s",this->fixed_frame.c_str());

  if(msg->header.frame_id == this->fixed_frame)
  {
    if(!this->we_habe_tracks_of_other_person_companion_){
	    this->we_habe_tracks_of_other_person_companion_=true;
		  //ROS_INFO(" WE HAVE TRACKS OF dabo");	
	    this->other_person_companion_tracks_mutex_enter();
	    //std::vector<SdetectionObservation> obs_dabo;
	    this->obs_other_person_companion_.clear();
	    std::vector<double> cov;
	    cov.resize(16,0.0);
	    //ROS_INFO("AkpLocalPlanner::tracks_callback: msg->detection.size() = %d",msg->detection.size());
	    for( unsigned int i = 0; i< msg->detection.size(); ++i)
	    {
	     //ROS_INFO(" [IN OBS OTHER PERSON COMPANION TRACKS!!!] AkpLocalPlanner::tracks_callback: id = %d, (x,y) = (%f, %f), (vx,vy) = (%f, %f), prob=%f", msg->detection[i].id, msg->detection[i].position.x, msg->detection[i].position.y, msg->detection[i].velocity.x, msg->detection[i].velocity.y, msg->detection[i].probability);
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
	      this->obs_other_person_companion_.push_back(SdetectionObservation(msg->detection[i].id, msg->header.stamp.toSec(),
		                                  msg->detection[i].position.x,  msg->detection[i].position.y ,
		                                  msg->detection[i].velocity.x, msg->detection[i].velocity.y, cov));
	    }
	    //this->planner_.update_scene(this->obs);//filter = false
	    this->other_person_companion_tracks_mutex_exit();
	    //ROS_INFO("AkpLocalPlanner::tracks_callback: Exit"); 
    }
  }
  else
  {
    ROS_ERROR("AkpLocalPlanner(other_person_companion)::tracks_callback: tracks are in %s frame instead of %s", msg->header.frame_id.c_str(), this->fixed_frame.c_str());
  }
  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->other_person_companion_tracks_mutex_exit();
}

void AkpLocalPlanner::other_person_companion_tracks_mutex_enter(void)
{
  pthread_mutex_lock(&this->other_person_companion_tracks_mutex_);
}

void AkpLocalPlanner::other_person_companion_tracks_mutex_exit(void)
{
  pthread_mutex_unlock(&this->other_person_companion_tracks_mutex_);
}










void AkpLocalPlanner::other_person_destination_tracks_callback(const iri_perception_msgs::detectionArray::ConstPtr& msg)
{
  //ROS_INFO("AkpLocalPlanner::other_person_destination_tracks_callback: New Message Received");
  
    if(!this->we_habe_destinations_of_other_person_companion_){
	    this->we_habe_destinations_of_other_person_companion_=true;
		  //ROS_INFO(" WE HAVE TRACKS OF dabo");	
	    this->other_person_destination_tracks_mutex_enter();
	    //std::vector<SdetectionObservation> obs_dabo;
	    this->destinations_other_person_companion_.clear();
	    //ROS_INFO("AkpLocalPlanner::tracks_callback: msg->detection.size() = %d",msg->detection.size());
	    for( unsigned int i = 0; i< msg->detection.size(); ++i)
	    {
	     //ROS_INFO(" [IN destinations OTHER PERSON COMPANION!!!] AkpLocalPlanner::tracks_callback: id = %d, (x,y) = (%f, %f)", msg->detection[i].id, msg->detection[i].position.x, msg->detection[i].position.y);	      
	      this->destinations_other_person_companion_.push_back(Sdestination(msg->detection[i].id,
		    msg->detection[i].position.x,  msg->detection[i].position.y));
	    }
	    this->other_person_destination_tracks_mutex_exit();
    }
  
}






void AkpLocalPlanner::other_person_destination_tracks_mutex_enter(void)
{
  pthread_mutex_lock(&this->other_person_destination_tracks_mutex_);
}

void AkpLocalPlanner::other_person_destination_tracks_mutex_exit(void)
{
  pthread_mutex_unlock(&this->other_person_destination_tracks_mutex_);
}

