#include "akp_local_planner_alg_node.h"
#include <pluginlib/class_list_macros.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <wiimote/State.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_DECLARE_CLASS(iri_akp_local_planner, 
                        AkpLocalPlanner, 
                        AkpLocalPlanner, 
                        nav_core::BaseLocalPlanner)
AkpLocalPlanner::AkpLocalPlanner(void) :
  fscan_received(false), 
  rscan_received(false), 
  planner_(6.0,600, Cplan_local_nav::F_RRT_GC_alpha),  // horitzont time=5.0, max_iter=500; d_horitzon= 5*V_robot(0.75)
  costmap_ros_(NULL), 
  tf_(NULL), 
  initialized_(false), 
  setup_(false), 
  vis_mode_(1),
  frozen_mode_(false), 
  move_base(true),
  text_markers_old_size_(0),
  slicing_path_diff_orientation_(20.0),
  move_base_client_("move_base", true), // add to send goal
  robot_ini_pose_x_(1.0),
  robot_ini_pose_y_(-15.0),
  robot_ini_pose_theta_(1.57),
  debug_antes_subgoals_entre_AKP_goals_(false),
  get_scout_results_doGoal_(false),
  external_goal_(true),
  debug_real_test_companion_(false),
  debug_real_test_companion_robot_(false),
  iter(0),
  use_default_wii_button_(true),
  use_default_PS3_button_(true),
  fuera_bolitas_goals_companion_markers_(true), // a false, no veo los dos goals diferentes de companion y follow the path!
  check_execution_times_(false),
  iterator_facke_(1),
  flag_play_change_id_(false)
 // prueba_count(0)
{
  pthread_mutex_init(&this->planner_mutex_,NULL);
  pthread_mutex_init(&this->fscan_mutex_,NULL);
  pthread_mutex_init(&this->rscan_mutex_,NULL);
  pthread_mutex_init(&this->tracks_mutex_,NULL);
  pthread_mutex_init(&this->odom_mutex_,NULL);
  pthread_mutex_init(&this->params_values_mutex_,NULL);
 // pthread_mutex_init(&this->cmd_vel_stop_mutex_,NULL);
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

  // change initial position
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(robot_ini_pose_x_, robot_ini_pose_y_, 0.0) );
  ROS_INFO("IN tibi! AkpLocalPlanner");
  //ROS_INFO("(ROS) AkpLocalPlanner::fill_robot_companion_markers: transform.pose.position.y=%f", transform.pose.position.y);   
  //ROS_INFO("(ROS) AkpLocalPlanner:: 1");     
  transform.setRotation( tf::Quaternion(0, 0, robot_ini_pose_theta_) ); // msg->theta= buscar y guardar por arriba orientación robot.
  //ROS_INFO("(ROS) AkpLocalPlanner:: 2");     
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/tibi/base_footprint"));//br.sendTransform(tf::StampedTransfor

  // init PS3 and Wii variables:
  this->human_is_alive_=false;
  this->vt_max            = 0.5;
  this->vr_max            = 0.5;
  this->trans_speed_scale = 0.1;
  this->rot_speed_scale   = 0.1;
  this->cancel_goal       = false;

  // stop to evaluate costs:
  this->current_state = HSQ_IT;
    //this->current_state = HSQ_STOP;
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

/*void AkpLocalPlanner::cmd_vel_stop_mutex_enter(void){
  pthread_mutex_lock(&this->cmd_vel_stop_mutex_);
}
void AkpLocalPlanner::cmd_vel_stop_mutex_exit(void){
  pthread_mutex_unlock(&this->cmd_vel_stop_mutex_);
}*/

// mutex for Wii and PS3 comandaments.

void AkpLocalPlanner::joy_mutex_enter(void) 
{ 
  pthread_mutex_lock(&this->joy_mutex_); 
} 

void AkpLocalPlanner::joy_mutex_exit(void) 
{ 
  pthread_mutex_unlock(&this->joy_mutex_); 
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
 // pthread_mutex_destroy(&this->cmd_vel_stop_mutex_);
}

void AkpLocalPlanner::init()
{

  ROS_INFO("IN tibi! AkpLocalPlanner init()");
  // publishers
  this->cmd_vel_publisher_ = this->public_node_handle_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  this->markers_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("planner_markers", 100);
  this->markers_publisher_m2_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("planner_markers_m2", 100);
  this->markers_publisher_comp_markers_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("planner_markers_comp_markers", 100);

  this->markers_publisher_people_prediction_time_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("planner_markers_people_predictions", 100);

  this->cost_params_publisher_ = this->public_node_handle_.advertise<std_msgs::Float64MultiArray>("cost_values", 10);
    //publisher teleop, para cambiar mis parametros internos.

 //this->status_init_publisher_ = this->public_node_handle_.advertise<iri_perception_msgs::restartSim>("status_init", 10);

  // subscrivers
  this->fscan_subscriber_   = this->public_node_handle_.subscribe<sensor_msgs::LaserScan>("front_scan" , 1, boost::bind(&AkpLocalPlanner::fscan_callback, this, _1));
  this->rscan_subscriber_   = this->public_node_handle_.subscribe<sensor_msgs::LaserScan>("rear_scan" , 1,  boost::bind(&AkpLocalPlanner::rscan_callback, this, _1));
  this->odom_subscriber_   = this->public_node_handle_.subscribe<nav_msgs::Odometry>("odom"  , 1, boost::bind(&AkpLocalPlanner::odom_callback, this, _1));
  this->tracks_subscriber_ = this->public_node_handle_.subscribe<iri_perception_msgs::detectionArray>("tracks", 1, boost::bind(&AkpLocalPlanner::tracks_callback, this, _1));
  this->params_values_subscriber_ = this->public_node_handle_.subscribe<std_msgs::Float64MultiArray>("params", 1, boost::bind(&AkpLocalPlanner::params_values_callback, this, _1));


  // [init subscribers] for Wii and PS3 comandaments
  this->joy_subscriber_    = this->public_node_handle_.subscribe("joy", 1, &AkpLocalPlanner::joy_callback, this);
  pthread_mutex_init(&this->joy_mutex_,NULL);
    
  // To stop code for evaluate the costs.
  this->cmd_vel_stop_subscriber_ = this->public_node_handle_.subscribe("cmd_vel_stop", 1, &AkpLocalPlanner::cmd_vel_stop_callback, this);

  
  // [init clients]
  init_simulations_client_ = this->public_node_handle_.serviceClient<iri_perception_msgs::InitialiceSim>("init_simulations");

 //make_plan_srv_ = this->private_node_handle_.advertiseService("make_plan", &NavfnROS::makePlanService, this);
	

  get_plan_client_ = this->public_node_handle_.serviceClient<nav_msgs::GetPlan>("/tibi/move_base/NavfnROS/make_plan");

  Float64_msg_.data.resize(8,0.0);//TODO cuantos elementos?
 ROS_INFO("OUT tibi! AkpLocalPlanner init()");
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
    private_nh.getParam("robot", this->robot_);  // tipo de robot, no pose inicial.
    double horizon_time, v_max, w_max, av_max, av_break, aw_max, platform_radii;
    double cost_distance,cost_orientation,cost_w_robot,cost_w_people,cost_time, cost_obs, cost_old_path, cost_l_minima;//, cost_companion; (NO TODO)
    int mode, nvertex;
    this->planner_.set_dt(0.2);//TODO depends upon the move_base frame_rate
    
    //robot frames
    this->fixed_frame = "/map";
    //this->robot_frame  = "/" + this->robot_ + "/base_link";
     // this->robot_frame  = "/tibi/base_footprint";
    this->robot_frame  = "/tibi/base_link";

    private_nh.getParam("move_base", this->move_base);

    // si quisiera hackear el move_base, sería poner esto:
    this->move_base=true;  //ely! si quiero que al principio este parado hay que quitarlo
    //ROS_INFO("AkpLocalPlanner::initialize: this->move_base=",this->move_base);
    
    if(debug_antes_subgoals_entre_AKP_goals_){
      std::cout << " AkpLocalPlanner::initialize this->move_base ="<< this->move_base  << std::endl;
    }
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
//     private_nh.getParam("cost_companion",  cost_companion); //son parametros de planing, no los costes que salen fuera. NO necesito ninguno para companion!!! // NO TODO: add companion cost
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
    ROS_INFO("(robot) AkpLocalPlanner::initialize: esfm_to_person_companion_lambda=%f", params[1]);
    ROS_INFO("AkpLocalPlanner::initialize: esfm_to_person_companion_A=%f", params[2]);
    ROS_INFO("AkpLocalPlanner::initialize: esfm_to_person_companion_B=%f", params[3]);
    ROS_INFO("AkpLocalPlanner::initialize: esfm_companion_d=%f", params[4]);
    this->planner_.set_sfm_to_person_companion( params );


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

    dsrv_ = new dynamic_reconfigure::Server<iri_akp_local_planner_companion::AkpLocalPlannerConfig>(private_nh);
    dynamic_reconfigure::Server<iri_akp_local_planner_companion::AkpLocalPlannerConfig>::CallbackType cb = boost::bind(&AkpLocalPlanner::reconfigureCallback, this, _1, _2);
    dsrv_->setCallback(cb);
    initialized_ = true;

    // Do initialization goal!
    this->actual_goal.x=this->robot_pose_.x;
    this->actual_goal.y=this->robot_pose_.y;
   // this->doGoal(this->actual_goal);
 
    ROS_INFO("AkpLocalPlanner::initialize: Fin initialize()");

  }
  else
    ROS_WARN("AkpLocalPlanner::initialize: This planner has already been initialized, you can't call it twice, doing nothing");

  // primer send a goal!!!(ely)= ya no computa, ya aparece en la posición que le toca.== Si se mueve de posicion inicial, descomentar esto.
  /*Spoint ini_goal(-1.0,-8.0,ros::Time::now().toSec());
  this->actual_goal=ini_goal;
  this->doGoal(this->actual_goal);  */

  //this->planner_.update_scene_companion_simulation(this->obs,true); // initialice scene

}

bool AkpLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{ 



//ROS_INFO( " computeVelocityCommands ");

    this->planner_mutex_enter();
double START_time_secs =ros::Time::now().toSec();
clock_t robot_node_start, robot_node_end; // clocks, pueden ser utiles... pero cambiando nombres, al menos en los scouts.
	robot_node_start = clock();

 //ROS_INFO("AkpLocalPlanner::computeVelocityCommands");

   double t = ros::Time::now().toSec();
    Spose best_next_pose;
    //Cperson_abstract::companion_reactive reactive_;
    // bool robot_plan_succed =  this->planner_.robot_plan(best_next_pose);
    bool robot_plan_succed=false;

  /*switch(this->current_state)
  {
    case HSQ_STOP:

      this->robot_pose_.v=0.000001;
      this->robot_pose_.w=0;
      robot_plan_succed=true;
      this->move_base=false;
      best_next_pose=this->robot_pose_;
      cmd_vel.linear.x  = 0.000001;
      cmd_vel.angular.z = 0.0;
      this->we_have_companion_person_=false;


    break;

    case HSQ_IT:*/
    
    //function_joy_teleop_mainNodeThread(); // wii and PS3 comandaments // todo, mirar si es necesaria que creo que no...

    if(check_execution_times_){
      std::cout <<std::endl<<std::endl;
      ROS_INFO("ENTRO EN AkpLocalPlanner::computeVelocityCommands!!!");
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




//ROS_INFO( " before get laser ");
double START_time_secs_read_scan =ros::Time::now().toSec();
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
	double END_time_secs_read_scan;
	if(check_execution_times_){ 
	END_time_secs_read_scan =ros::Time::now().toSec();
	std::cout<< "IIIIIIIIIIIIII ros_time_node read_laser_points=" <<END_time_secs_read_scan-START_time_secs_read_scan<< std::endl;
	  }

double START_time_secs_read_laser_scan =ros::Time::now().toSec();
    this->planner_.read_laser_scan( this->laser_points );
	double END_time_secs_read_laser_scan;
	if(check_execution_times_){ 
	END_time_secs_read_laser_scan =ros::Time::now().toSec();
	std::cout<< "IIIIIIIIIIIIII ros_time_node read_laser_scan function=" <<END_time_secs_read_laser_scan-START_time_secs_read_laser_scan<< std::endl;
	  }

//ROS_INFO( "after get laser ");
    //this->planner_.read_laser_scan_companion( this->laser_points );

   
    /*if(prueba_count>20){
      this->robot_pose_.x=0;
      this->robot_pose_.y=0;
    }else{
      prueba_count++;
    }*/

 //update robot position
//ROS_INFO( " before update_robot ");
double START_time_secs_updateRobot =ros::Time::now().toSec();
    this->odom_mutex_enter();
    this->planner_.update_robot(this->robot_pose_);
    this->odom_mutex_exit();
//ROS_INFO( " after update_robot ");
	double END_time_secs_updateRobot;
	if(check_execution_times_){ 
	END_time_secs_updateRobot =ros::Time::now().toSec();
	std::cout<< "IIIIIIIIIIIIII ros_time_node updateRobot=" <<END_time_secs_updateRobot-START_time_secs_updateRobot<< std::endl;
	  }
	    
    this->we_have_companion_person_=false;
    //update people observations
    this->tracks_mutex_enter();

    
    if(debug_real_test_companion_){
      ROS_INFO("[ROS ROBOT] AkpLocalPlanner::computeVelocityCommands: (antes update scene) this->obs.size()=%lu=",this->obs.size());
      for(unsigned int u=0;u<this->obs.size();u++){
        ROS_INFO( "this->obs[%d].vx=%f; .vy=%f",u,this->obs[u].vx,this->obs[u].vy);
      }
    }

//ROS_INFO( " before update_scene ");
	double START_time_secs_updateScene =ros::Time::now().toSec();
  this->planner_.update_scene(this->obs,this->we_have_companion_person_);
	double END_time_secs_updateScene;
//ROS_INFO( "after update_scene ");

	if(check_execution_times_){ 
	END_time_secs_updateScene =ros::Time::now().toSec();
	std::cout<< "IIIIIIIIIIIIII ros_time_node updateScene=" <<END_time_secs_updateScene-START_time_secs_updateScene<< std::endl;
	}

//}
  /*  if(this->current_state==HSQ_STOP){
      this->we_have_companion_person_=false;
    }

    if(this->current_state==HSQ_IT){
       this->current_state = HSQ_STOP;
    }*/
   


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
   
    //this->actual_goal=this->planner_.get_robot_goal();
    //this->doGoal(this->actual_goal);  
    if(debug_real_test_companion_){
	ROS_INFO( "AkpLocalPlanner::computeVelocityCommands: this->we_have_companion_person_= %s ", this->we_have_companion_person_ ? "true" : "false");

    }
    //planner iteration
 
double START_time_secs10;

//ROS_INFO( " before server simulation ");

double START_time_secs_server_sim =ros::Time::now().toSec();

    if(this->we_have_companion_person_){

	
            Action_ROS_=this->planner_.get_state_Action(); // //
              ROS_INFO("AkpLocalPlanner::computeVelocityCommands: Action_ROS_=",Action_ROS_);

		 Actual_case_ROS_=this->planner_.get_actual_case();
		if(simulation_){
 /* INI server to use the same state in people simulation and robot */
      // first know what state is now!!!

 /*      status_init_msg_.initial_position.clear();
                  status_init_msg_.initial_position.resize(3);
                  status_init_msg_.orientation.clear();
                  status_init_msg_.orientation.resize(3);

                  status_init_msg_.initial_position[0].x=0.0;     // id=0; empty, no simulated person with this id   
                  status_init_msg_.initial_position[0].y=0.0;  
                  status_init_msg_.initial_position[0].z=0.0;          
                  status_init_msg_.orientation[0]=0.0;
  
                  status_init_msg_.initial_position[1].x=0.1;  // person companion position          
                 status_init_msg_.initial_position[1].y=0.5;  
                  status_init_msg_.initial_position[1].z=0.0;
                  status_init_msg_.orientation[1]=0.0;

                  status_init_msg_.initial_position[2].x=17.0;  // person goal position
                  status_init_msg_.initial_position[2].y=10.5; 
                  status_init_msg_.initial_position[2].z=0.0;
                  status_init_msg_.orientation[2]=0.0;

                this->status_init_publisher_.publish(this->status_init_msg_);*/


                //ROS_INFO("AkpLocalPlanner::computeVelocityCommands: enter Action_ROS_=Cplan_local_nav::START=Action_ROS_=%d=",Action_ROS_);
 
               // + generar el mensaje a enviar al server.
              Spoint person_companion_point1=this->planner_.get_SIM_initial_person_companion_pose1();
              Spoint person_companion_point2=this->planner_.get_SIM_initial_person_companion_pose2();

              Spoint person_goal_point=this->planner_.get_SIM_initial_person_goal_pose_actual();


                if(Actual_case_ROS_==Cplan_local_nav::case0){
                 // ROS_INFO(" [ROBOT] AkpLocalPlanner::computeVelocityCommands: case0");
                  init_simulations_srv_.request.init.act_case=0;
                }else if(Actual_case_ROS_==Cplan_local_nav::case1){
                  //ROS_INFO(" [ROBOT] AkpLocalPlanner::computeVelocityCommands: case1");
                  init_simulations_srv_.request.init.act_case=1;
                }else if(Actual_case_ROS_==Cplan_local_nav::case2){
                 // ROS_INFO(" [ROBOT] AkpLocalPlanner::computeVelocityCommands: case2");
                  init_simulations_srv_.request.init.act_case=2;
                }

            

              init_simulations_srv_.request.init.state=0;

             //  ROS_INFO("AkpLocalPlanner::computeVelocityCommands: (1) init_simulations_client_.call");
              init_simulations_srv_.request.init.initial_position.clear();
              init_simulations_srv_.request.init.initial_position.resize(3);
              init_simulations_srv_.request.init.orientation.clear();
              init_simulations_srv_.request.init.orientation.resize(3);

           init_simulations_srv_.request.init.initial_position[0].x=person_companion_point1.x;  // person companion position
              //  ROS_INFO("AkpLocalPlanner::computeVelocityCommands: (2) init_simulations_client_.call");
              init_simulations_srv_.request.init.initial_position[0].y=person_companion_point1.y; 
              //ROS_INFO("AkpLocalPlanner::computeVelocityCommands: (3) init_simulations_client_.call");
              init_simulations_srv_.request.init.initial_position[0].z=0.0;
              //ROS_INFO("AkpLocalPlanner::computeVelocityCommands: (4) init_simulations_client_.call");
              init_simulations_srv_.request.init.orientation[0]=0.0;

              //  ROS_INFO("AkpLocalPlanner::computeVelocityCommands: (5) init_simulations_client_.call");
              init_simulations_srv_.request.init.initial_position[1].x=person_companion_point2.x;  // person companion position
              //  ROS_INFO("AkpLocalPlanner::computeVelocityCommands: (6) init_simulations_client_.call");
              init_simulations_srv_.request.init.initial_position[1].y=person_companion_point2.y; 
             // ROS_INFO("AkpLocalPlanner::computeVelocityCommands: (7) init_simulations_client_.call");
              init_simulations_srv_.request.init.initial_position[1].z=0.0;
              //ROS_INFO("AkpLocalPlanner::computeVelocityCommands: (8) init_simulations_client_.call");
             // init_simulations_srv_.request.init.initial_position[1].velocity.x=0.4; 
             // init_simulations_srv_.request.init.initial_position[1].velocity.y=0.2;  
            //  init_simulations_srv_.request.init.initial_position[1].velocity.z=0.0;
            //   std::vector<double> cov;
            //    for(unsigned int it=0;it<35;it++){
            //      cov.push_back(0.0);
             //   }
           //   init_simulations_srv_.request.init.initial_position[1].covariances=cov;
           //   init_simulations_srv_.request.init.initial_position[1].id=1;
            //  init_simulations_srv_.request.init.initial_position[1].probability=0.7;
              init_simulations_srv_.request.init.orientation[1]=0.0;

              init_simulations_srv_.request.init.initial_position[2].x=person_goal_point.x;  // person goal position
              init_simulations_srv_.request.init.initial_position[2].y=person_goal_point.y; 
              init_simulations_srv_.request.init.initial_position[2].z=0.0;
             /* init_simulations_srv_.request.init.initial_position[2].velocity.x= 1; 
              init_simulations_srv_.request.init.initial_position[2].velocity.y=0.5; 
              init_simulations_srv_.request.init.initial_position[2].velocity.z=0.0;
              init_simulations_srv_.request.init.initial_position[2].covariances=cov;
              init_simulations_srv_.request.init.initial_position[2].id=2;
              init_simulations_srv_.request.init.initial_position[2].probability=0.8;*/
              init_simulations_srv_.request.init.orientation[2]=3.14;


   
            if(Action_ROS_==Cplan_local_nav::START){ // solicita al server del People_simulation que resetee a las personas.
              // solicitar que reinicie el server.
              
             bool call_ok=init_simulations_client_.call(init_simulations_srv_);
             /* if(call_ok){
                ROS_INFO("AkpLocalPlanner::computeVelocityCommands: out init_simulations_client_.call; TRUE");
              }else{
                ROS_INFO("AkpLocalPlanner::computeVelocityCommands: out init_simulations_client_.call; FALSE");
              }*/
            }else{
                init_simulations_srv_.request.init.state=1; // case ITER
                bool call_ok=init_simulations_client_.call(init_simulations_srv_);
            }
 	}
	double END_time_secs_server_sim ;
	if(check_execution_times_){
	END_time_secs_server_sim =ros::Time::now().toSec();
	std::cout<< "IIIIIIIIIIIIII ros_time_node server simulation=" <<END_time_secs_server_sim-START_time_secs_server_sim<< std::endl;
	}
 /* Fin server to use the same state in people simulation and robot */


      //ROS_INFO( "  if(this->we_have_companion_person_) ANTES this->planner_.robot_plan_companion2(best_next_pose,reactive_); ");
     // robot_plan_succed =  this->planner_.robot_plan_companion2(best_next_pose,reactive_);
      
      double ros_time_to_sec=ros::Time::now().toSec()/(60*60);
      //ROS_INFO( "IIIIIIIIIIIIIIIIIII  ros::Time::now().toSec()= ",ros::Time::now().toSec());
      this->planner_.set_ros_time_to_sec(ros_time_to_sec);

  //ROS_INFO( " before robot_plan_companion3 ");
	double START_time_secs2 =ros::Time::now().toSec();
      robot_plan_succed =  this->planner_.robot_plan_companion3(best_next_pose,reactive_);
	double END_time_secs3;
  //ROS_INFO( " after robot_plan_companion3 ");
	if(check_execution_times_){
		END_time_secs3 =ros::Time::now().toSec();
		std::cout<< "IIIIIIIIIIIIII after plan3 ros_time_node=" <<END_time_secs3-START_time_secs<< std::endl;
		std::cout<< "IIIIIIIIIIIIII only plan3 ros_time_node=" <<END_time_secs3-START_time_secs2<< std::endl;
	}


	
      this->move_base=true;
      /* if(this->move_base==true){
       ROS_INFO( "(tiene que ser true) this->move_base=true");
      }else{
        ROS_INFO( "(tiene que ser true) this->move_base=false");
      }*/
      /*if(this->we_have_companion_person_==true){
        ROS_INFO( "(tiene que ser true) this->we_have_companion_person_=true");
      }else{
        ROS_INFO( "(tiene que ser true) this->we_have_companion_person_=false");
      }*/

      if(debug_real_test_companion_robot_){
        ROS_INFO( "CASO BUENO!!! true this->we_have_companion_person_[in_robot_plan]  (this->move_base=true)! calc robot_plan_companion2 best_next_pose.v=%f",best_next_pose.v);
        ROS_INFO( "2 [in_robot_plan] (ROS) best_next_pose.w=%f",best_next_pose.w);
      }
      
    }else{

      ROS_INFO( " [don't have companion person!] SET V=0 and w=0 of robot ");

      this->robot_pose_.v=0.000000000001;
      this->robot_pose_.w=0;
      robot_plan_succed=true;
      this->move_base=false;
      best_next_pose=this->robot_pose_;
     
      if(debug_real_test_companion_robot_){
        ROS_INFO( "3 (else) false No have companion person  (this->move_base=false)!  robot_plan_succed=true; calc NO robot_plan_companion2 best_next_pose.v=%f",best_next_pose.v);
        ROS_INFO( "3 [in_robot_plan] (ROS) best_next_pose.w=%f",best_next_pose.w);
        if(this->move_base==true){
          ROS_INFO( " 3 (tiene que ser false) this->move_base=true");
        }else{
          ROS_INFO( " 3 (tiene que ser false) this->move_base=false");
        }
        if(this->we_have_companion_person_==true){
          ROS_INFO( "3 (tiene que ser false) this->we_have_companion_person_=true");
        }else{
          ROS_INFO( "3 (tiene que ser false) this->we_have_companion_person_=false");
        }
      }

    // ROS_INFO( " this->robot_pose_.v=  %f, this->robot_pose_.w=%f ",  this->robot_pose_.v,this->robot_pose_.w );
    //robot_plan_succed=false;
    }

    if(debug_real_test_companion_){
      ROS_INFO("AkpLocalPlanner::computeVelocityCommands: reactive_=%d=",reactive_);
    }
    //save_best_next_pose_= // solo para ver el marker de a donde intenta ir.

    if(debug_antes_subgoals_entre_AKP_goals_){
      ROS_INFO( "best_next_pose.x=  %f ", best_next_pose.x );
      ROS_INFO( "best_next_pose.y=  %f ", best_next_pose.y ); 
    }

    if(debug_real_test_companion_){
      ROS_INFO( "robot plan  %s in time %f", robot_plan_succed ? "true" : "false", ros::Time::now().toSec()-t );
    }
    best_next_pose_companion_markers_=best_next_pose; // for companion! (luego para cambiar el goal del robot, tendrá que ser aquí, donde cambie su posicion con el angulo entre robot y persona)

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
      //robot_plan_succed = false;
      ROS_INFO( "robot plan would be invalidated"); // es de Gonzalo, no se que de minimos para saltarselos, pero a mi no me va bien con el robot!
    }

START_time_secs10 =ros::Time::now().toSec();
    if(this->move_base)
    {
      cmd_vel.linear.x  = best_next_pose.v;
      cmd_vel.angular.z = best_next_pose.w;

      if(debug_real_test_companion_robot_){
        if(!robot_plan_succed){
          ROS_INFO( "CASO BUENO (pero robot_plan_succed=false (this->move_base=true) (ROS) best_next_pose.v=%f",best_next_pose.v);
          ROS_INFO( "4 if(!robot_plan_succed)if (ROS) best_next_pose.w=%f",best_next_pose.w);
        }else{
          ROS_INFO( "CASO BUENO pero robot_plan_succed=true (this->move_base=true) (ROS) best_next_pose.v=%f",best_next_pose.v);
          ROS_INFO( "4.2 (ROS) best_next_pose.w=%f",best_next_pose.w);
        }
      }

    }
    else
    {
   	ROS_INFO( "[don't have companion person!] this->move_base=false set v=0 and w=0"); // es de Gonzalo, no se que de minimos para saltarselos, pero a mi no me va bien con el robot!
 
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
    //t = ros::Time::now().toSec();//just to measure the drawing of markers
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
      
      if(fuera_bolitas_goals_companion_markers_){
        MarkerArray_msg_comp_markers_.markers.push_back( best_next_pose_robot_companion_markers_ );
      }
	double END_time_secs5;
	if(check_execution_times_){
		END_time_secs5 =ros::Time::now().toSec();
		std::cout<< "IIIIIIIIIIIIII purria code after all plan3 ros_time_node=" <<END_time_secs5-START_time_secs10<< std::endl;
	}

//ROS_INFO( " before get paths size ");

double START_time_secs_path =ros::Time::now().toSec();
///// INI GET size of the path /////
double global_plan_lenght=0;
  for( unsigned int i=1; i < global_plan_.size(); ++i )
  {
    /*fake orientation message, filled with constant q=[0,0,0,1] o.O
    ROS_INFO( "quaternion entering = (%f, %f, %f, %f)" ,global_plan_[i].pose.orientation.x,
                      global_plan_[i].pose.orientation.y,
                      global_plan_[i].pose.orientation.z,
                      global_plan_[i].pose.orientation.w );*/
    
    //differential in 2d
    double dy = global_plan_[i].pose.position.y - global_plan_[i-1].pose.position.y;
    double dx = global_plan_[i].pose.position.x - global_plan_[i-1].pose.position.x;
   
    //ROS_INFO("2d processing = %f", theta);
	global_plan_lenght=global_plan_lenght + sqrt(dy*dy+dx*dx);
	
  }


 unsigned int i=1;
  double global_path_ini_orientation=atan2( global_plan_[i].pose.position.y - global_plan_[i-1].pose.position.y, global_plan_[i].pose.position.x - global_plan_[i-1].pose.position.x );
  i=global_plan_.size(); // i=global_plan_.size(); o i=global_plan_.size()-1; => comprobar.
  double global_path_final_orientation=atan2( global_plan_[i].pose.position.y - global_plan_[i-1].pose.position.y, global_plan_[i].pose.position.x - global_plan_[i-1].pose.position.x );
 // ROS_INFO("global_plan_lenght= %f", global_plan_lenght);
   this->planner_.set_value_distance_global_path(global_plan_lenght);
   this->planner_.set_global_path_ini_orientation(global_path_ini_orientation);
   this->planner_.set_global_path_final_orientation(global_path_final_orientation);
///// FIN GET size of the path /////
	double END_time_secs_path;
	if(check_execution_times_){
		END_time_secs_path =ros::Time::now().toSec();
		std::cout<< "IIIIIIIIIIIIII time get path distance=" <<END_time_secs_path-START_time_secs_path<< std::endl;
	}
	//SpointV_cov actual_robot_position=this->planner_.get_robot_pose();


	//ROS_INFO( " before fill marcker ");

	START_time_secs_fill_markers =ros::Time::now().toSec();
      fill_best_path_2d();
      // }
      //ROS_INFO( "entering scene markers" );
      fill_scene_markers();
      if( vis_mode_ > 1 )//normal mode 2d features
      {
	ROS_DEBUG(" before fill_forces_markers();");
        fill_forces_markers();
	ROS_DEBUG(" before fill_laser_obstacles();");
        fill_laser_obstacles();
	ROS_DEBUG(" before fill_people_prediction_markers_path_points();");
        fill_people_prediction_markers_path_points(); // to see the target person predicted path
	ROS_DEBUG(" before fill_planning_markers_2d;");
        //if(reactive_==Cperson_abstract::Akp_planning){// unico caso en que hay planning y se pueden generar los markers del best path.
          fill_planning_markers_2d(); // seguramente que tambien da problemas...
          ROS_DEBUG(" before fill_planning_markers_2d();");
	fill_people_prediction_markers_3d(); // provisional. TODO. QUITAr
        //} 
        //if(reactive_==Cperson_abstract::Akp_planning){// unico caso en que hay planning y se pueden generar los markers del best path.
	ROS_DEBUG(" before fill_people_prediction_markers_3d();");
         fill_planning_markers_3d(); // mirar...   
	ROS_DEBUG(" before fill_planning_markers_3d();");

      }
      if ( vis_mode_ > 2 )//super mode 3d features
      {
        //fill_people_prediction_markers_2d();
	ROS_DEBUG(" before fill_people_prediction_markers_3d();");
        fill_people_prediction_markers_3d();
	ROS_DEBUG("fill_planning_markers_3d();");
        //if(reactive_==Cperson_abstract::Akp_planning){// unico caso en que hay planning y se pueden generar los markers del best path.
          fill_planning_markers_3d(); // mirar...
	ROS_DEBUG("after fill_planning_markers_3d();");
        //}
      }
      if ( vis_mode_ > 3 )//super mode, plooting potentials and so on
      {
        //TODO
      }
      //fill_robot_companion_markers(); //  Quitado para quitar coste computacional. de momento inhabilitado, pq ya se ve desde robot y persona simulada directamente.

     // fill_people_prediction_markers_2d_companion(); Quitado para quitar coste computacional


      // fill_people_prediction_markers_3d_companion();
      //if(reactive_==Cperson_abstract::Akp_planning){// unico caso en que hay planning y se pueden generar los markers del best path.
      //fill_planning_markers_3d_companion();

    } 
    fill_test_markers(); // made by ely
	double END_time_secs_fill_markers;
	if(check_execution_times_){
		END_time_secs_fill_markers =ros::Time::now().toSec();
		std::cout<< "IIIIIIIIIIIIII time fill markers=" <<END_time_secs_fill_markers-START_time_secs_fill_markers<< std::endl;
	}
	double START_time_secs_publish_markers =ros::Time::now().toSec();
    this->markers_publisher_.publish(this->MarkerArray_msg_);
    this->markers_publisher_comp_markers_.publish(this->MarkerArray_msg_comp_markers_);
    this->markers_publisher_m2_.publish(this->MarkerArray_msg_m2_);
    this->markers_publisher_people_prediction_time_.publish(this->MarkerArray_msg_people_prediction_time_);
	double END_time_secs_publish_markers;
	if(check_execution_times_){
	END_time_secs_publish_markers =ros::Time::now().toSec();
	std::cout<< "IIIIIIIIIIIIII time publish markers=" <<END_time_secs_publish_markers-START_time_secs_publish_markers<< std::endl;
	}
    // publish robot costs TODO to be deprecated
    //double work_robot, work_persons;
    //this->planner_.get_navigation_instant_work( work_robot, work_persons );
    //Float64_msg_.data[0] = work_robot;
    //ROS_DEBUG("FoceLocalPlanner::computeVelocityCommands: cmd_vel=(%f,%f)", cmd_vel.linear.x,  cmd_vel.angular.z);
    
    // publish navigation costs dist[0], orient[1], robot[2], ppl[3], obst[4]
    //ROS_INFO( "entering costs" );
	double START_time_secs_publish_planerInfo =ros::Time::now().toSec();
    std::vector<double> costs;  // TODO: en teoria el companion_cost. esta aquí add de dentro, pero no segura...
    this->planner_.get_navigation_cost_values( costs);  // publica cost values del best path
    Float64_msg_.data = costs;
    this->planner_.get_navigation_mean_cost_values( costs); // publica las medias de esta iteración
    Float64_msg_.data.insert( Float64_msg_.data.end(), costs.begin(), costs.end() );
    this->planner_.get_navigation_std_cost_values( costs); // publica las std de esta iteración
    Float64_msg_.data.insert( Float64_msg_.data.end(), costs.begin(), costs.end() );
    this->cost_params_publisher_.publish(this->Float64_msg_);
	double END_time_secs_publish_planerInfo;
	if(check_execution_times_){
		END_time_secs_publish_planerInfo =ros::Time::now().toSec();
		std::cout<< "IIIIIIIIIIIIII time publish node topics=" <<END_time_secs_publish_planerInfo-START_time_secs_publish_planerInfo<< std::endl;
	}
    //ROS_INFO( "current markers = %f", ros::Time::now().toSec()-t);
 
    //ROS_INFO("AkpLocalPlanner::ComputeVelocites: Exit"); 

    if(debug_real_test_companion_){
      ROS_INFO( "time of processing (todo planing==computeVelocityCommands)= %f", (ros::Time::now()-now).toSec());
      std::cout <<std::endl<<std::endl;
    }
   // this->current_state = HSQ_STOP;
  /* break;

  }*/


	double START_do_goal_time_secs =ros::Time::now().toSec();
	//this->doGoal(); // mirar si aquí o más arriba...
	//bool we_have_targ_pers=;
	// ROS_INFO( " before doGoal");
	if(this->planner_.get_we_have_target_person()){
		this->doGoal2();
		//this->doGoal();
  	}else{
		this->doGoal(); // lanza el goal, pero antes comprueba que ese goal sea accesible al robot
  	}

	double END_do_goal_time_secs;
	if(check_execution_times_){
		END_do_goal_time_secs =ros::Time::now().toSec();
		std::cout<< "IIIIIIIIIIIIII ros_time_do_goal=" <<END_do_goal_time_secs-START_do_goal_time_secs<< std::endl;
	}

	robot_node_end = clock();
	unsigned int clock_start4;
	unsigned int clock_end4;
	double END_time_secs;
	if(check_execution_times_){
		robot_node_end = clock();
		clock_start4= (unsigned int) robot_node_start;
		clock_end4= (unsigned int) robot_node_end;
		END_time_secs =ros::Time::now().toSec();
		std::cout<< "IIIIIIIIIIIIII ros_time_node=" <<END_time_secs-START_time_secs<< std::endl;

	}

 	//ROS_INFO( " FIN ROS NODE ");
   this->planner_mutex_exit();
  return robot_plan_succed;
}

bool AkpLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
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
  // if(debug_antes_subgoals_entre_AKP_goals_){
  //ROS_INFO("AkpLocalPlanner::setPlan: global_plan_ points.x= %f ; points.y= %f", orig_global_plan.back().pose.position.x, orig_global_plan.back().pose.position.y);
  //ROS_INFO("AkpLocalPlanner::setPlan: global_plan_ points %d", uint(this->global_plan_.size()));
  //}
  // ROS_INFO("AkpLocalPlanner::setPlan: global_plan_ points %d", uint(this->global_plan_.size()));
  // slice global plan into subgoals
  if ( goal_providing_mode_ == AkpLocalPlanner::Slicing_global )
    slice_plan( );

  //else , there is no need to do anything
  return true;
}

void AkpLocalPlanner::slice_plan()
{

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
      //check if it near the previous on and then discard
      dx = sliced_global_plan_.back().pose.position.x - global_plan_[i].pose.position.x;
      dy = sliced_global_plan_.back().pose.position.y - global_plan_[i].pose.position.y;
      accumulated_orientation = 0.0;
      Spoint robot_act_global_goal_=Spoint(global_plan_[i].pose.position.x, global_plan_[i].pose.position.y);
      double  d_horitzon= planner_.get_horizon_time()*planner_.get_robot2()->get_v_max ();//V_robot(0.75)

      //ROS_INFO("[slice_plan() => d_horitzon=%f; robot_vel_max=%f",d_horitzon,planner_.get_robot2()->get_v_max ());

      if ( (dx*dx + dy*dy > 1.0) && (this->robot_pose_.distance( robot_act_global_goal_ ) > d_horitzon) )
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

  //this->planner_mutex_enter();
  if(debug_antes_subgoals_entre_AKP_goals_){
    ROS_INFO("ENTRO EN AkpLocalPlanner::isGoalReached!!!");
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
  
    //ROS_INFO("ENTRO EN AkpLocalPlanner::isGoalReached!!! (caso else)");
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

  

 //this->robot_goal_.x=this->global_plan_.back().pose.position.x;
 //this->robot_goal_.y=this->global_plan_.back().pose.position.y;


 // if((reactive_==Cperson_abstract::Akp_planning)&&external_goal_){
   if(debug_antes_subgoals_entre_AKP_goals_){
      ROS_INFO("AkpLocalPlanner::isGoalReached: ENTRO EN set_robot_external_goal ROS!!! this->robot_goal_.x=%f ; this->robot_goal_.y= %f",this->robot_goal_.x,this->robot_goal_.y);
      ROS_INFO("AkpLocalPlanner::isGoalReached: ENTRO EN set_robot_external_goal ROS!!! reactive_=%d=",reactive_);
  }
    //this->planner_.set_robot_external_goal(this->robot_goal_); // para darle el goal externo desde el rviz!!!!
   // this->planner_.set_robot_external_goal_fix(this->robot_goal_); // para darle el goal externo desde el rviz!!!!
	
	this->planner_mutex_enter();
	this->planner_.set_robot_goal(this->robot_goal_);
	this->planner_.set_robot_goal_person_goal_global_plan_IN_robot_VERSION(this->robot_goal_);
	this->planner_mutex_exit();
 // }
  if(this->robot_pose_.distance( this->robot_goal_ ) < this->xy_goal_tolerance
     && this->robot_pose_.v < this->v_goal_tolerance)
  {
    ok=true;
    this->planner_.set_final_goal_reached_in_node(true);
	// TODO: set robot desired velocity ==0, when robot just over the desired final goal!!!
	this->robot_pose_.v=0;
	this->robot_pose_.w=0;
   // ROS_INFO("AkpLocalPlanner::isGoalReached: GOAL REACHED x,y: %f, %f at distance %f", robot_goal_.x, robot_goal_.y, this->robot_pose_.distance( this->robot_goal_ ));
  }else{
    this->planner_.set_final_goal_reached_in_node(false);
  }

  //this->planner_mutex_exit();


	ok=false; // TODO: Move base = true when I want!!! (Case STOP of the target person!)
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
  
  header_point_.header= scan.header;  // variable for companion (ely)

  std::vector<Spoint> points;
  points.clear();
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
    //ROS_INFO("PeopleSimulationAlgNode::scan2points: 2 target_time=%f",target_time.toSec());
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
  //ROS_INFO("AkpLocalPlanner::odom_callback: New Message Received");

  
  Spose actual_odom_robot_pose;
  actual_odom_robot_pose.v = msg->twist.twist.linear.x;
  actual_odom_robot_pose.w = msg->twist.twist.angular.z;

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
      actual_odom_robot_pose.x = poseOut.pose.position.x;
      actual_odom_robot_pose.y = poseOut.pose.position.y;
      actual_odom_robot_pose.time_stamp = poseOut.header.stamp.toSec();

      //vector of the orientation
      poseIn.pose.position.x = 1.0;
      this->tf_->transformPose(target_frame, poseIn, poseOut);
      actual_odom_robot_pose.theta = atan2(poseOut.pose.position.y - this->robot_pose_.y , poseOut.pose.position.x - this->robot_pose_.x);
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

this->odom_mutex_enter();
	this->robot_pose_=actual_odom_robot_pose;
  this->odom_mutex_exit();
  //ROS_INFO("AkpLocalPlanner::odom_callback: Exit");
}

void AkpLocalPlanner::tracks_callback(const iri_perception_msgs::detectionArray::ConstPtr& msg) 
{
 ROS_INFO("AkpLocalPlanner::tracks_callback: New Message Received"); 
  
  //ROS_INFO("AkpLocalPlanner::tracks_callback: msg->header.frame_id= %s",msg->header.frame_id.c_str());
  //ROS_INFO("AkpLocalPlanner::tracks_callback: this->fixed_frame= %s",this->fixed_frame.c_str());

  if(msg->header.frame_id == this->fixed_frame)
  {
    ROS_INFO("AkpLocalPlanner::tracks_callback: (transform ok)"); 
    this->tracks_mutex_enter();
    //std::vector<SdetectionObservation> obs;
    this->obs.clear();
    std::vector<double> cov;
    cov.resize(16,0.0);
    //ROS_INFO("AkpLocalPlanner::tracks_callback: msg->detection.size() = %d",msg->detection.size());
    for( unsigned int i = 0; i< msg->detection.size(); ++i)
    {
     // ROS_INFO(" [IN ROBOT TRACKS!!!] AkpLocalPlanner::tracks_callback: id = %d, (x,y) = (%f, %f), (vx,vy) = (%f, %f), prob=%f", msg->detection[i].id, msg->detection[i].position.x, msg->detection[i].position.y, msg->detection[i].velocity.x, msg->detection[i].velocity.y, msg->detection[i].probability);
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


// subscriber for the wii and PS3 comandaments


void AkpLocalPlanner::joy_callback(const sensor_msgs::Joy::ConstPtr& msg) 
{
  this->alg_.lock(); // mirar mutex...
  static std::vector<int> last_buttons=msg->buttons;
  std::vector<int> current_buttons = msg->buttons;
  std::vector<float> current_axes = msg->axes;
  
  this->reset_joy_watchdog();
  
  if(/*this->config_.joy_type==0 &&*/ (msg->axes.size()==29 && msg->buttons.size()==17) || (msg->axes.size()==27 && msg->buttons.size()==19) ) //ps3 bluetooth or usb-cable
    this->usePs3(current_buttons, last_buttons, current_axes);
  else if(/*this->config_.joy_type==1 &&*/ msg->axes.size()==3 && msg->buttons.size()==11) //wiimote number of axes and buttons
    this->useWii(current_buttons, last_buttons);
  else
    ROS_ERROR("PlatformTeleopAlgNode::joy_callback: unrecognized joy msg (%ld axes, %ld buttons)", msg->axes.size(), msg->buttons.size());
  
  last_buttons=current_buttons;
  
  this->alg_.unlock();
}


/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void AkpLocalPlanner::reconfigureCallback(iri_akp_local_planner_companion::AkpLocalPlannerConfig &config, uint32_t level)
{

  this->planner_mutex_enter();
  // ROS_INFO(" INI AkpLocalPlanner::reconfigureCallback");
  std::vector<double> params(5,0.0);
  if(setup_ && config.restore_defaults)
  {
    config = default_config_;
    //Avoid looping
    config.restore_defaults = false;
  }
  if(!setup_)
  {

	  if((flag_play_change_id_)&&(id_person_companion_!=config.id_person_companion)){
		  config.id_person_companion=id_person_companion_;
		  flag_play_change_id_=false;
	  }

    this->robot_goal_force_marker       = config.robot_goal_force_marker;
    this->robot_person_forces_marker    = config.robot_person_forces_marker;
    this->robot_obstacles_forces_marker = config.robot_obstacles_forces_marker;
    this->robot_resultant_force_marker  = config.robot_resultant_force_marker;

    default_config_ = config;
    setup_ = true;
    // change (ely) para puentear lo de move_base a true! (que me deje dar goals desde el principio!)
    

    this->move_base = config.move_base;
    //this->move_base = true;//config.move_base;
    if(debug_antes_subgoals_entre_AKP_goals_){
      std::cout << " AkpLocalPlanner::reconfigureCallback this->move_base ="<< this->move_base  << std::endl;
    }
    this->plan_mode_ = (Cplan_local_nav::plan_mode) config.plan_mode;
    this->planner_.set_planning_mode(plan_mode_);
    this->planner_.set_distance_mode((Cplan_local_nav::distance_mode)config.distance_mode );
    this->planner_.set_global_mode((Cplan_local_nav::global_mode)config.global_mode );
    this->planner_.set_number_of_vertex(config.number_vertex);
    // ROS_INFO("\n\n\n numer of vertex = %d\n\n\n\n" , config.number_vertex);
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

    // ini companion config variables.

    this->planner_.set_id_person_companion(config.id_person_companion);
    this->planner_.set_robot_person_proximity_distance(config.proximity_distance_between_robot_and_person);
    this->planner_.set_proximity_distance_tolerance(config.proximity_distance_tolerance);
    this->planner_.set_additional_distance_companion_sphere(config.add_dist_companion_sphere);
    this->planner_.set_proximity_goals_robot_and_person(config.proximity_goals_robot_and_person_x,config.proximity_goals_robot_and_person_y);
    this->planner_.set_offset_attractive(config.offset_attractive_state);
    this->planner_.set_force_obs_max(config.force_obs_max_x,config.force_obs_max_y);
    // this->planner_.set_real_companion_angle(config.real_companion_angle);
    this->planner_.set_person_goal_percentage(config.person_goal_percentage);
    this->planner_.set_id_person_companion_Cprediction_bhmip(config.id_person_companion);
    id_person_companion_=config.id_person_companion;
    this->planner_.set_overpas_obstacles_behind_person(config.overpas_obstacles_behind_person);
    this->planner_.set_anisotropy_threshold(config.anisotropy_threshold);
    this->planner_.set_max_d_to_detect_laser_obs(config.detection_laser_obstacle_distances);    
    external_goal_=config.external_goal;
    if(config.external_goal==true){
       this->planner_.set_companion_same_person_goal(false);
    }else{
      this->planner_.set_companion_same_person_goal(true);
    }
    this->planner_.set_max_asos_point_to_person(config.in_max_asos_point_to_person);

    this->planner_.set_save_results_in_file(config.save_results_in_file);
    this->planner_.set_meters_to_goal_to_save_results_in_file(config.metros_al_goal);
    this->planner_.set_mode_velocity(config.mode_velocity);
    this->planner_.set_results_filename(config.results_filename);
    this->planner_.set_evaluate_costs_filename(config.evaluate_costs_filename);
    this->planner_.set_evaluate_change_distance_and_angle_companion_filename(config.evaluate_change_distance_and_angle_companion_filename);
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

    //this->planner_.set_out_index_step(config.out_index_step);
    this->planner_.set_out_index_step_companion_goal(config.out_index_step_companion_goal);
    this->planner_.set_out_index_step_final_dest_goal(config.out_index_stepfinal_dest_goal);

    this->planner_.only_comp_people_vel_and_robot_poses(config.debug_only_robot_point_comp_person_point);
    this->planner_.set_person_radi_amp(config.person_radi_amp);
    this->planner_.set_obstacle_radi_amp(config.obstacle_radi_amp);
    this->planner_.set_ini_vel_to_increment_angle(config.ini_vel_to_increment_angle);
    // end companion config variables.

    // Ini config PS3 and Wii (comandaments) to select up or down the value of Beta, for learning the wights alpha and beta of the forces.
    this->vt_max            = config.max_translation_speed;
    this->vr_max            = config.max_rotation_speed;
    this->trans_speed_scale = config.translation_increment_step;
    this->rot_speed_scale   = config.rotation_increment_step;
    this->cancel_goal       = config.cancel_goal;
    this->config_            = config;
    this->use_default_wii_button_=config.use_default_wii_button;
    this->use_default_PS3_button_=config.use_default_PS3_button;
    // end config PS3 and Wii (comandaments) to select up or down the value of Beta, for learning the wights alpha and beta of the forces.

    // change betha_companion from reconfigure
    this->planner_.set_beta_companion(config.betha_companion);
    this->planner_.set_alpha_companion(1-config.betha_companion);

    // 
    this->planner_.set_new_B_force_face_person(config.set_new_B_force_face_person);
    this->planner_.set_new_A_force_face_person(config.set_new_A_force_face_person);

    // get person goal ID
    id_person_goal_=config.person_goal_id;
    this->planner_.set_id_person_goal(config.person_goal_id);

    // some parameters descent gradient meet person goal
    this->planner_.set_constant_time_steps(config.constant_time_steps);
    this->planner_.set_num_max_iter_constan_time_steps(config.num_max_iter_constan_time_steps);
    this->planner_.set_lambda_descent_gradient(config.lambda_descent_gradient);
    this->planner_.set_get_geometrical_time_to_meet_person(config.get_geometrical_time_to_meet_person);
    this->planner_.set_max_dist_to_go_behind(config.max_dist_to_go_behind);
    this->planner_.set_distance_between_companion_and_target_to_face_person(config.dis_COMP_and_targ_to_face_person);
    this->planner_.set_distance_between_robot_and_target_to_face_person(config.dis_ROBOT_and_targ_to_face_person);

    // restart real robot, to reestart experiment and save results in a new file
    this->planner_.set_restart_real(config.restart_real);
    if(config.restart_real_data_txt){
      this->planner_.set_restart_real_data_txt();
    }
    this->planner_.set_change_sim(config.change_sim);
    simulation_=config.change_sim;
    this->planner_.set_change_sim_target_per(config.change_sim_target_per);
    this->planner_.set_change_external_to_stop_case(config.change_external_to_stop_case);

    this->planner_.set_inc_m_dist_between_group_and_pers_to_calc_dyn_goal_Prediction_behaviour(config.set_inc_dist_calculate_dyn_goal_less_comp_time);
    this->planner_.set_debug_bad_goal_calc(config.debug_inc_dist_to_dyn_goal);

    this->planner_.set_change_goal_of_the_error(config.in_change_goal_of_the_error);

   this->planner_.set_distance_faceperson_between_robot_and_target_person(config.in_dist_facepers_between_robot_and_target_person);
   this->planner_.set_distance_faceperson_between_companion_and_target_person(config.in_dist_facepers_between_companion_and_target_person);	
   this->planner_.set_distance_faceperson_between_robot_and_companion_person(config.in_dist_facepers_between_robot_and_companion_person);



   if(config.select_closest_person_as_companion){// to select the closest person as person companion
        unsigned int id_min_dist_per;
	      config.select_closest_person_as_companion=false;
	      if(this->obs.empty()){
	        Spoint actual_robot_pose;   //observation[i].id 
	        actual_robot_pose.x=this->robot_pose_.x;
	        actual_robot_pose.y=this->robot_pose_.y;
	        actual_robot_pose.time_stamp=this->robot_pose_.time_stamp;

	        double min_distance;
	    
	        for(unsigned int t=0; t<this->obs.size(); t++){
		        Spoint actual_person;
		        actual_person.x=this->obs[t].x;
		        actual_person.y=this->obs[t].y;
		        actual_person.time_stamp=this->obs[t].time_stamp;
	
		        if(t==0){
		          min_distance=actual_person.distance(actual_robot_pose);
		          id_min_dist_per=this->obs[t].id;
		        }
		        if(actual_person.distance(actual_robot_pose)<min_distance){
		          min_distance=actual_person.distance(actual_robot_pose);
		          id_min_dist_per=this->obs[t].id;
		        }
	        }
	      this->planner_.set_id_person_companion(id_min_dist_per);
	      id_person_companion_=id_min_dist_per;
		    config.id_person_companion=id_min_dist_per;
		    ROS_INFO("case select near person as companion! id_person=%d",id_min_dist_per);
	     }
	
    }
    ROS_INFO("out case select near person as companion! id_person");
    /*this->planner_.set_case_stop_giro(config.case_stop_giro);
    this->planner_.set_incremento_giro_positivo(config.incremento_giro_positivo);
    this->planner_.set_incremento_giro_negativo(config.incremento_giro_negativo);
    */

    //this->planner_mutex_exit();

  }
  else if(setup_)
  {
    

	  if((flag_play_change_id_)&&(id_person_companion_!=config.id_person_companion)){
		  config.id_person_companion=id_person_companion_;
		  flag_play_change_id_=false;
	  }
    //this->planner_mutex_enter();
    this->move_base = config.move_base;
    //this->move_base = true;//config.move_base;
    if(debug_antes_subgoals_entre_AKP_goals_){
      std::cout << " AkpLocalPlanner::reconfigureCallback this->move_base ="<< this->move_base  << std::endl;
    }
    this->plan_mode_ = (Cplan_local_nav::plan_mode) config.plan_mode;
    this->planner_.set_planning_mode(plan_mode_);
    this->planner_.set_distance_mode((Cplan_local_nav::distance_mode)config.distance_mode );
    this->planner_.set_global_mode((Cplan_local_nav::global_mode)config.global_mode );
    this->planner_.set_number_of_vertex(config.number_vertex);
    if(debug_antes_subgoals_entre_AKP_goals_){
      ROS_INFO("\n\n\n numer of vertex = %d\n\n\n\n" , config.number_vertex);
    }
    this->planner_.set_horizon_time( config.horizon_time );
    if(debug_antes_subgoals_entre_AKP_goals_){
      ROS_INFO("\n\n\n horizon_time = %f\n\n\n\n" , config.horizon_time );
    }
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

    // ini companion config variables.

    this->planner_.set_id_person_companion(config.id_person_companion);
    this->planner_.set_robot_person_proximity_distance(config.proximity_distance_between_robot_and_person);
    this->planner_.set_proximity_distance_tolerance(config.proximity_distance_tolerance);
    this->planner_.set_additional_distance_companion_sphere(config.add_dist_companion_sphere);
    this->planner_.set_proximity_goals_robot_and_person(config.proximity_goals_robot_and_person_x,config.proximity_goals_robot_and_person_y);
    this->planner_.set_offset_attractive(config.offset_attractive_state);
    this->planner_.set_force_obs_max(config.force_obs_max_x,config.force_obs_max_y);
    // this->planner_.set_real_companion_angle(config.real_companion_angle);
    this->planner_.set_person_goal_percentage(config.person_goal_percentage);
    this->planner_.set_id_person_companion_Cprediction_bhmip(config.id_person_companion);
    id_person_companion_=config.id_person_companion;
    this->planner_.set_overpas_obstacles_behind_person(config.overpas_obstacles_behind_person);
    this->planner_.set_anisotropy_threshold(config.anisotropy_threshold);
    this->planner_.set_max_d_to_detect_laser_obs(config.detection_laser_obstacle_distances);
    external_goal_=config.external_goal;
    if(config.external_goal==true){
       this->planner_.set_companion_same_person_goal(false);
    }else{
      this->planner_.set_companion_same_person_goal(true);
    }
    this->planner_.set_max_asos_point_to_person(config.in_max_asos_point_to_person);
    this->planner_.set_save_results_in_file(config.save_results_in_file);
    this->planner_.set_meters_to_goal_to_save_results_in_file(config.metros_al_goal);
    this->planner_.set_mode_velocity(config.mode_velocity);
    this->planner_.set_results_filename(config.results_filename);
    this->planner_.set_evaluate_costs_filename(config.evaluate_costs_filename);
    this->planner_.set_evaluate_change_distance_and_angle_companion_filename(config.evaluate_change_distance_and_angle_companion_filename);
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
        //this->planner_.set_out_index_step(config.out_index_step);
    this->planner_.set_out_index_step_companion_goal(config.out_index_step_companion_goal);
    this->planner_.set_out_index_step_final_dest_goal(config.out_index_stepfinal_dest_goal);
    this->planner_.only_comp_people_vel_and_robot_poses(config.debug_only_robot_point_comp_person_point);
    this->planner_.set_person_radi_amp(config.person_radi_amp);
    this->planner_.set_obstacle_radi_amp(config.obstacle_radi_amp);
    this->planner_.set_ini_vel_to_increment_angle(config.ini_vel_to_increment_angle);
    // end companion config variables.

    // Ini config PS3 and Wii (comandaments) to select up or down the value of Beta, for learning the wights alpha and beta of the forces.
    this->vt_max            = config.max_translation_speed;
    this->vr_max            = config.max_rotation_speed;
    this->trans_speed_scale = config.translation_increment_step;
    this->rot_speed_scale   = config.rotation_increment_step;
    this->cancel_goal       = config.cancel_goal;
    this->config_            = config;
    // end config PS3 and Wii (comandaments) to select up or down the value of Beta, for learning the wights alpha and beta of the forces.

    // change betha_companion from reconfigure
    this->planner_.set_beta_companion(config.betha_companion);
    this->planner_.set_alpha_companion(1-config.betha_companion);

    // get person goal ID
    id_person_goal_=config.person_goal_id;
    this->planner_.set_id_person_goal(config.person_goal_id);

    // some parameters descent gradient meet person goal
    this->planner_.set_constant_time_steps(config.constant_time_steps);
    this->planner_.set_num_max_iter_constan_time_steps(config.num_max_iter_constan_time_steps);
    this->planner_.set_lambda_descent_gradient(config.lambda_descent_gradient);
    this->planner_.set_get_geometrical_time_to_meet_person(config.get_geometrical_time_to_meet_person);
    this->planner_.set_max_dist_to_go_behind(config.max_dist_to_go_behind);
    this->planner_.set_distance_between_companion_and_target_to_face_person(config.dis_COMP_and_targ_to_face_person);
    this->planner_.set_distance_between_robot_and_target_to_face_person(config.dis_ROBOT_and_targ_to_face_person);
    this->planner_.set_change_sim(config.change_sim);
    simulation_=config.change_sim;
    this->planner_.set_change_sim_target_per(config.change_sim_target_per);

    // restart real robot, to reestart experiment and save results in a new file
    this->planner_.set_restart_real(config.restart_real);

    if(config.restart_real_data_txt){
      this->planner_.set_restart_real_data_txt();
    }
    this->planner_.set_change_external_to_stop_case(config.change_external_to_stop_case);
    this->planner_.set_inc_m_dist_between_group_and_pers_to_calc_dyn_goal_Prediction_behaviour(config.set_inc_dist_calculate_dyn_goal_less_comp_time);
    this->planner_.set_debug_bad_goal_calc(config.debug_inc_dist_to_dyn_goal);
    this->planner_.set_change_goal_of_the_error(config.in_change_goal_of_the_error);
    this->planner_.set_distance_faceperson_between_robot_and_target_person(config.in_dist_facepers_between_robot_and_target_person);
    this->planner_.set_distance_faceperson_between_companion_and_target_person(config.in_dist_facepers_between_companion_and_target_person);	
    this->planner_.set_distance_faceperson_between_robot_and_companion_person(config.in_dist_facepers_between_robot_and_companion_person);

    if(config.select_closest_person_as_companion){// to select the closest person as person companion
      config.select_closest_person_as_companion=false;

  	  unsigned int id_min_dist_per;
	    ROS_INFO("7777777777777777777777 case select near person as companion! id_person  this->obs.size()=%lu",this->obs.size());
	    if(!this->obs.empty()){
	
	      Spoint actual_robot_pose;   //observation[i].id 
	      actual_robot_pose.x=this->robot_pose_.x;
	      actual_robot_pose.y=this->robot_pose_.y;
	      actual_robot_pose.time_stamp=this->robot_pose_.time_stamp;

	      double min_distance;
	    
	      for(unsigned int t=0; t<this->obs.size(); t++){
		      Spoint actual_person;
		      actual_person.x=this->obs[t].x;
		      actual_person.y=this->obs[t].y;
		      actual_person.time_stamp=this->obs[t].time_stamp;

		      if(t==0){
		        min_distance=actual_person.distance(actual_robot_pose);
		        id_min_dist_per=this->obs[t].id;
		      }
		      if(actual_person.distance(actual_robot_pose)<min_distance){
		        min_distance=actual_person.distance(actual_robot_pose);
		        id_min_dist_per=this->obs[t].id;
		      }
	      }

	      this->planner_.set_id_person_companion(id_min_dist_per);
	      id_person_companion_=id_min_dist_per;
	      config.id_person_companion=id_min_dist_per;

 	    }
	    ROS_INFO("777777777777777777777 case select near person as companion! id_person=%d",id_min_dist_per);
    }


	this->planner_.set_plan_local_nav_group_go_to_interact_with_other_person(config.conf_bool_group_go_to_interact_with_other_person);
    /*this->planner_.set_case_stop_giro(config.case_stop_giro);
    this->planner_.set_incremento_giro_positivo(config.incremento_giro_positivo);
    this->planner_.set_incremento_giro_negativo(config.incremento_giro_negativo);
*/
   
  }

//std::cout << " OUT!!!! AkpLocalPlanner::reconfigureCallback this->move_base ="<< this->move_base  << std::endl;
   this->planner_mutex_exit();
}

//copied from goal_functions.cpp in base_local_planner
bool AkpLocalPlanner::transformGlobalPlan(const tf::TransformListener& tf, 
                                            const std::vector<geometry_msgs::PoseStamped>& global_plan, 
                                            const costmap_2d::Costmap2DROS& costmap, 
                                            const std::string& global_frame, 
                                            std::vector<geometry_msgs::PoseStamped>& transformed_plan)
{
  if(debug_antes_subgoals_entre_AKP_goals_){
    ROS_INFO("AkpLocalPlanner::transformGlobalPlan!!!!");
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

   if(debug_antes_subgoals_entre_AKP_goals_){
   ROS_INFO("AkpLocalPlanner::transformPose!!!");
  }
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
  //ROS_INFO("AkpLocalPlanner::publishPlan!!!");

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
  pred_traj_marker_.color.a = 0.05;
  pred_traj_marker_.color.r = 0.3;
  pred_traj_marker_.color.g = 1.0;
  pred_traj_marker_.color.b = 0.3;
  pred_traj_marker_.pose.orientation.w = 1.0;
  
  //target marker of the best path, projected in 2d
  pred_traj2d_marker_.header.frame_id = this->fixed_frame;
  pred_traj2d_marker_.ns = "pred2d";
  pred_traj2d_marker_.type = visualization_msgs::Marker::CYLINDER;
  pred_traj2d_marker_.action = visualization_msgs::Marker::ADD;
  pred_traj2d_marker_.lifetime = ros::Duration(1.23f);
  pred_traj2d_marker_.scale.z = 0.05;
  pred_traj2d_marker_.color.a = 0.05;
  pred_traj2d_marker_.color.r = 0.3;
  pred_traj2d_marker_.color.g = 1.0;
  pred_traj2d_marker_.color.b = 0.3;
  pred_traj2d_marker_.pose.orientation.w = 1.0;
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
  robot_goal_marker_.ns = "scene_goal1";
  robot_goal_marker_.type = visualization_msgs::Marker::CYLINDER;
  robot_goal_marker_.action = visualization_msgs::Marker::ADD;
  robot_goal_marker_.lifetime = ros::Duration(1.3f);
  robot_goal_marker_.color.a = 1.0;
  robot_goal_marker_.color.r = 0.0; // 0.9
  robot_goal_marker_.color.g = 0.0; // 0.3
  robot_goal_marker_.color.b = 0.0; // 0.3
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


  // robot_real_goal_marker!!!

  robot_goal_marker2_.header.frame_id = this->fixed_frame;
  robot_goal_marker2_.ns = "scene_goal";
  robot_goal_marker2_.type = visualization_msgs::Marker::CYLINDER;
  robot_goal_marker2_.action = visualization_msgs::Marker::ADD;
  robot_goal_marker2_.lifetime = ros::Duration(1.3f);
  robot_goal_marker2_.color.a = 1.0;
  robot_goal_marker2_.color.r = 0.0;
  robot_goal_marker2_.color.g = 0.0;
  robot_goal_marker2_.color.b = 1.0;
  robot_goal_marker2_.scale.x = 0.4;
  robot_goal_marker2_.scale.y = 0.4;
  robot_goal_marker2_.scale.z = 1.0;
  robot_goal_marker2_.pose.position.z = 0.1;
  robot_goal_marker2_.pose.orientation.w = 1.0;


  // robot_real_goal_marker!!!

  robot_goal_marker3_.header.frame_id = this->fixed_frame;
  robot_goal_marker3_.ns = "scene_goal2";
  robot_goal_marker3_.type = visualization_msgs::Marker::CYLINDER;
  robot_goal_marker3_.action = visualization_msgs::Marker::ADD;
  robot_goal_marker3_.lifetime = ros::Duration(1.3f);
  robot_goal_marker3_.color.a = 0.0;
  robot_goal_marker3_.color.r = 0.5;
  robot_goal_marker3_.color.g = 1.0;
  robot_goal_marker3_.color.b = 1.0;
  robot_goal_marker3_.scale.x = 1.4;
  robot_goal_marker3_.scale.y = 1.4;
  robot_goal_marker3_.scale.z = 2.0;
  robot_goal_marker3_.pose.position.z = 0.1;
  robot_goal_marker3_.pose.orientation.w = 1.0;


  // workspace marker
  workspace_marker_.header.frame_id = this->fixed_frame;
  workspace_marker_.ns = "scene_w";
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

  //marker due to obstacles in the scene (black)
  force_obstacle_marker_ = force_marker_;
  force_obstacle_marker_.color.r = 0.0;
  force_obstacle_marker_.color.g = 0.0;
  force_obstacle_marker_.color.b = 0.0;

  // force of interaction with robot (purple)
  force_int_robot_marker_ = force_marker_;
  force_int_robot_marker_.color.r = 0.26;
  force_int_robot_marker_.color.g = 0.0;
  force_int_robot_marker_.color.b = 0.66;

  // marker force companion (cian)
  force_companion_marker_ = force_marker_;
  force_companion_marker_.color.r = 0.51;
  force_companion_marker_.color.g = 0.243;
  force_companion_marker_.color.b = 0.255;

  
  //text marker
  text_marker_.header.frame_id = this->fixed_frame;
  text_marker_.ns = "text";
  text_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker_.action = visualization_msgs::Marker::ADD;
  text_marker_.lifetime = ros::Duration(1.3f);
  text_marker_.scale.z = 0.3;
  text_marker_.pose.position.z = 0.0;
  text_marker_.color.a = 1.0;

  // apartir de aquí ya no se publican!
  //person_companion marker, for know the person position in the simulation. 
  person_companion_marker_.header.frame_id = this->fixed_frame;
  person_companion_marker_.ns = "personCompanion";
  person_companion_marker_.type = visualization_msgs::Marker::CYLINDER;
  person_companion_marker_.action = visualization_msgs::Marker::ADD;
  person_companion_marker_.lifetime = ros::Duration(1.3f);
  person_companion_marker_.color.a = 1.0;
  person_companion_marker_.color.r = 0.5;
  person_companion_marker_.color.g = 0.0;
  person_companion_marker_.color.b = 1.0;
  person_companion_marker_.scale.x = 0.5;
  person_companion_marker_.scale.y = 0.5;
  person_companion_marker_.scale.z = 1.2;
  person_companion_marker_.pose.position.z = 0.6;
  person_companion_marker_.pose.orientation.w = 1.0;


  //Robot_companion marker, for know the person position in the simulation. 
  robot_companion_marker_.header.frame_id = this->fixed_frame;
  robot_companion_marker_.ns = "robotCompanion";
  robot_companion_marker_.type = visualization_msgs::Marker::CYLINDER;
  robot_companion_marker_.action = visualization_msgs::Marker::ADD;
  robot_companion_marker_.lifetime = ros::Duration(1.3f);
  robot_companion_marker_.color.a = 1.0;
  robot_companion_marker_.color.r = 0.0;
  robot_companion_marker_.color.g = 1.0;
  robot_companion_marker_.color.b = 0.0;
  robot_companion_marker_.scale.x = 0.5;
  robot_companion_marker_.scale.y = 0.5;
  robot_companion_marker_.scale.z = 1.2;
  robot_companion_marker_.pose.position.z = 0.6;
  robot_companion_marker_.pose.orientation.w = 1.0;

  //center_companion marker, for know the center position of the companion robot-person in the simulation.  
  center_companion_marker_.header.frame_id = this->fixed_frame;
  center_companion_marker_.ns = "centerCompanion";
  center_companion_marker_.type = visualization_msgs::Marker::SPHERE;
  center_companion_marker_.action = visualization_msgs::Marker::ADD;
  center_companion_marker_.lifetime = ros::Duration(1.3f);
  center_companion_marker_.color.a = 0.0;
  center_companion_marker_.color.r = 0.0;
  center_companion_marker_.color.g = 1.0;
  center_companion_marker_.color.b = 0.0;
  center_companion_marker_.scale.x = 0.3;
  center_companion_marker_.scale.y = 0.3;
  center_companion_marker_.scale.z = 0.3;
  center_companion_marker_.pose.position.z = 0.0;
  center_companion_marker_.pose.orientation.w = 1.0;



  // prueba! ver que goal es el (this->robot_goal_)
  robot_see_goal_marker_.header.frame_id = this->fixed_frame;
  robot_see_goal_marker_.ns = "robot_see_goal";
  robot_see_goal_marker_.type = visualization_msgs::Marker::SPHERE;
  robot_see_goal_marker_.action = visualization_msgs::Marker::ADD;
  robot_see_goal_marker_.lifetime = ros::Duration(1.3f);
  robot_see_goal_marker_.color.a = 1.0;
  robot_see_goal_marker_.color.r = 1.0; // rojo!
  robot_see_goal_marker_.color.g = 0.0;
  robot_see_goal_marker_.color.b = 0.0;
  robot_see_goal_marker_.scale.x = 0.7;
  robot_see_goal_marker_.scale.y = 0.7;
  robot_see_goal_marker_.scale.z = 0.7;
  robot_see_goal_marker_.pose.position.z = 0.0;
  robot_see_goal_marker_.pose.orientation.w = 1.0;

  // prueba! ver que goal es el (this->robot_goal_)
  person_90_degre_marker_.header.frame_id = this->fixed_frame;
  person_90_degre_marker_.ns = "person_90_degre";
  person_90_degre_marker_.type = visualization_msgs::Marker::SPHERE;
  person_90_degre_marker_.action = visualization_msgs::Marker::ADD;
  person_90_degre_marker_.lifetime = ros::Duration(1.3f);
  person_90_degre_marker_.color.a = 1.0;
  person_90_degre_marker_.color.r = 0.0; // esfera azul!
  person_90_degre_marker_.color.g = 0.0;
  person_90_degre_marker_.color.b = 1.0;
  person_90_degre_marker_.scale.x = 0.3;
  person_90_degre_marker_.scale.y = 0.3;
  person_90_degre_marker_.scale.z = 0.3;
  person_90_degre_marker_.pose.position.z = 0.0;
  person_90_degre_marker_.pose.orientation.w = 1.0;

  // Marker para ver a que puntpo intenta ir la fuerza companion, por si no lo estoy haciendo bien.
  force_companion_goal_marker_.header.frame_id = this->fixed_frame;
  force_companion_goal_marker_.ns = "person_companion_goal";
  force_companion_goal_marker_.type = visualization_msgs::Marker::SPHERE;
  force_companion_goal_marker_.action = visualization_msgs::Marker::ADD;
  force_companion_goal_marker_.lifetime = ros::Duration(1.3f);
  force_companion_goal_marker_.color.a = 1.0;
  force_companion_goal_marker_.color.r = 1.0; //esfera rojo!
  force_companion_goal_marker_.color.g = 0.0;
  force_companion_goal_marker_.color.b = 0.0;
  force_companion_goal_marker_.scale.x = 0.7;
  force_companion_goal_marker_.scale.y = 0.7;
  force_companion_goal_marker_.scale.z = 0.7;
  force_companion_goal_marker_.pose.position.z = 0.0;
  force_companion_goal_marker_.pose.orientation.w = 1.0;

  // Marker para ver a que punto intenta ir el robot en cada estep, por si no lo estoy haciendo bien.
  best_next_pose_robot_companion_markers_.header.frame_id = this->fixed_frame;
  best_next_pose_robot_companion_markers_.ns = "best_next_pose_robot_companion_markers";
  best_next_pose_robot_companion_markers_.type = visualization_msgs::Marker::SPHERE;
  best_next_pose_robot_companion_markers_.action = visualization_msgs::Marker::ADD;
  best_next_pose_robot_companion_markers_.lifetime = ros::Duration(1.3f);
  best_next_pose_robot_companion_markers_.color.a = 1.0;
  best_next_pose_robot_companion_markers_.color.r = 0.0;
  best_next_pose_robot_companion_markers_.color.g = 1.0; // esfera verde!
  best_next_pose_robot_companion_markers_.color.b = 0.0;
  best_next_pose_robot_companion_markers_.scale.x = 0.3;
  best_next_pose_robot_companion_markers_.scale.y = 0.3;
  best_next_pose_robot_companion_markers_.scale.z = 0.3;
  best_next_pose_robot_companion_markers_.pose.position.z = 0.0;
  best_next_pose_robot_companion_markers_.pose.orientation.w = 1.0;

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



// See goal to go to a concrete position respect to the person that the robot accompani. 
  robot_goal_to_position_respect_to_the_person.header.frame_id = this->fixed_frame;
  robot_goal_to_position_respect_to_the_person.ns = "goal_to_go_person_side";
  robot_goal_to_position_respect_to_the_person.type = visualization_msgs::Marker::SPHERE;
  robot_goal_to_position_respect_to_the_person.action = visualization_msgs::Marker::ADD;
  robot_goal_to_position_respect_to_the_person.lifetime = ros::Duration(1.3f);
  robot_goal_to_position_respect_to_the_person.color.a = 1.0;
  robot_goal_to_position_respect_to_the_person.color.r = 0.0; // esfera verde! es azul oscuro creo.
  robot_goal_to_position_respect_to_the_person.color.g = 1.0;
  robot_goal_to_position_respect_to_the_person.color.b = 1.0;
  robot_goal_to_position_respect_to_the_person.scale.x = 0.3;
  robot_goal_to_position_respect_to_the_person.scale.y = 0.3;
  robot_goal_to_position_respect_to_the_person.scale.z = 0.3;
  robot_goal_to_position_respect_to_the_person.pose.position.z = 0.0;
  robot_goal_to_position_respect_to_the_person.pose.orientation.w = 1.0;

// See goal to go follow the path
  robot_goal_to_follow_the_path.header.frame_id = this->fixed_frame;
  robot_goal_to_follow_the_path.ns = "goal_to_follow_path";
  robot_goal_to_follow_the_path.type = visualization_msgs::Marker::SPHERE;
  robot_goal_to_follow_the_path.action = visualization_msgs::Marker::ADD;
  robot_goal_to_follow_the_path.lifetime = ros::Duration(1.3f);
  robot_goal_to_follow_the_path.color.a = 1.0;
  robot_goal_to_follow_the_path.color.r = 1.0; // esfera roja!
  robot_goal_to_follow_the_path.color.g = 0.0;
  robot_goal_to_follow_the_path.color.b = 1.0;
  robot_goal_to_follow_the_path.scale.x = 0.3;
  robot_goal_to_follow_the_path.scale.y = 0.3;
  robot_goal_to_follow_the_path.scale.z = 0.3;
  robot_goal_to_follow_the_path.pose.position.z = 0.0;
  robot_goal_to_follow_the_path.pose.orientation.w = 1.0;

// See the final goal to go, that combine these before goals with the interactions with other people and objects of the environment.
  final_robot_goal_act_iteration.header.frame_id = this->fixed_frame;
  final_robot_goal_act_iteration.ns = "final_goal_to_go";
  final_robot_goal_act_iteration.type = visualization_msgs::Marker::SPHERE;
  final_robot_goal_act_iteration.action = visualization_msgs::Marker::ADD;
  final_robot_goal_act_iteration.lifetime = ros::Duration(1.3f);
  final_robot_goal_act_iteration.color.a = 1.0;
  final_robot_goal_act_iteration.color.r = 0.0; // esfera azul!
  final_robot_goal_act_iteration.color.g = 0.0;
  final_robot_goal_act_iteration.color.b = 1.0;
  final_robot_goal_act_iteration.scale.x = 0.3;
  final_robot_goal_act_iteration.scale.y = 0.3;
  final_robot_goal_act_iteration.scale.z = 0.3;
  final_robot_goal_act_iteration.pose.position.z = 0.0;
  final_robot_goal_act_iteration.pose.orientation.w = 1.0;


// See the medium point between target person and person companion in the faceperson mode.
  medium_point_face_person_.header.frame_id = this->fixed_frame;
  medium_point_face_person_.ns = "medium_point_face_person";
  medium_point_face_person_.type = visualization_msgs::Marker::SPHERE;
  medium_point_face_person_.action = visualization_msgs::Marker::ADD;
  medium_point_face_person_.lifetime = ros::Duration(1.3f);
  medium_point_face_person_.color.a = 1.0;
  medium_point_face_person_.color.r = 0.0; // esfera verde!
  medium_point_face_person_.color.g = 1.0;
  medium_point_face_person_.color.b = 0.0;
  medium_point_face_person_.scale.x = 0.3;
  medium_point_face_person_.scale.y = 0.3;
  medium_point_face_person_.scale.z = 0.3;
  medium_point_face_person_.pose.position.z = 0.0;
  medium_point_face_person_.pose.orientation.w = 1.0;



  // see prediction path of the person target 
  
  pred_traj_point_marker_.header.frame_id = this->fixed_frame;
  pred_traj_point_marker_.ns = "trajectory_target_person";
  pred_traj_point_marker_.type = visualization_msgs::Marker::SPHERE;
  pred_traj_point_marker_.action = visualization_msgs::Marker::ADD;
  pred_traj_point_marker_.lifetime = ros::Duration(1.3f);
  pred_traj_point_marker_.color.a = 0.0;
  pred_traj_point_marker_.color.r = 0.0;
  pred_traj_point_marker_.color.g = 1.0;
  pred_traj_point_marker_.color.b = 0.0;
  pred_traj_point_marker_.scale.x = 0.3;
  pred_traj_point_marker_.scale.y = 0.3;
  pred_traj_point_marker_.scale.z = 0.3;
  pred_traj_point_marker_.pose.position.z = 0.0;
  pred_traj_point_marker_.pose.orientation.w = 1.0;

}

void AkpLocalPlanner::fill_my_covariance_marker( visualization_msgs::Marker& marker, const SpointV_cov& point , unsigned int track_id)
{
  marker.pose.position.x = point.x;
  marker.pose.position.y = point.y;
  marker.pose.position.z = 0.0;

  /*text_marker_.scale.x = 0.6;
  text_marker_.scale.y = 0.6;
  text_marker_.scale.z = 0.6;*/

           if(id_person_goal_==track_id){
               //   ROS_INFO( "CHANGE PERSON goal color marker");
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
              
               /* text_marker_.id=track_id;
                text_marker_.pose.position.x = point.x;
                text_marker_.pose.position.y = point.y;
                text_marker_.pose.position.z = 1.0;
                std::stringstream idText;
                unsigned int id_int=411;
                idText << id_int;
                text_marker_.text = idText.str();
                MarkerArray_msg_.markers.push_back( text_marker_ );*/


            }else if(id_person_companion_==track_id){
              // ROS_INFO( "CHANGE PERSON companion color marker");
              marker.color.r = 0.0;
              marker.color.g = 0.5;
              marker.color.b = 0.6;
                /*
                text_marker_.id=track_id;
                text_marker_.pose.position.x = point.x;
                text_marker_.pose.position.y = point.y;
                std::stringstream idText;
                unsigned int id_int=368;
                idText << id_int;
                text_marker_.text = idText.str();
                MarkerArray_msg_.markers.push_back( text_marker_ );*/
            }else{
              marker.color.r = 0.0;
              marker.color.g = 1.0;
              marker.color.b = 0.0;
            }

  /*text_marker_.scale.x = 0.0;
  text_marker_.scale.y = 0.0;
  text_marker_.scale.z = 0.3;*/

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


void AkpLocalPlanner::fill_my_prediction_points( visualization_msgs::Marker& marker, const SpointV_cov& point )
{
  marker.pose.position.x = point.x;
  marker.pose.position.y = point.y;
  marker.pose.position.z = 0.0;

  //plot covariances ellipses
 /* Eigen::Matrix2d cov;
  cov(0, 0) = point.cov.at(0);//xx
  cov(0, 1) = point.cov.at(1);
  cov(1, 0) = point.cov.at(4);
  cov(1, 1) = point.cov.at(5);//yy
*/
  //if((cov(0, 0)>1)||(cov(0, 1)>1)||(cov(1, 0)>1)||(cov(1, 1)>1)){
   // ROS_INFO(" cov(0, 0) = %f, cov(0, 1) = %f,cov(1, 0) = %f, cov(1, 1) = %f", cov(0, 0),cov(0, 1), cov(1, 0), cov(1, 1));
  // }
 /* Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);
  //eig.compute(cov);
  marker.scale.x = sqrt( eig.eigenvalues()[0] ) + 0.001;//major vap
  marker.scale.y = sqrt( eig.eigenvalues()[1] ) + 0.001;//minor vap
*/
  //if((marker.scale.x>1)||(marker.scale.y>1)){
   // ROS_INFO(" marker.scale.x = sqrt( eig.eigenvalues()[0] ) + 0.001= %f", marker.scale.x);
   // ROS_INFO(" marker.scale.y = sqrt( eig.eigenvalues()[1] ) + 0.001= %f", marker.scale.y);
  //}
 // double angle = atan2( eig.eigenvectors()(1,0), eig.eigenvectors()(0,0) );
 // marker.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
  
}


void AkpLocalPlanner::fill_forces_markers()
{
  const std::list<Cperson_abstract *>* person_list = planner_.get_scene( );
  Sforce force_to_goal, force_int_person , force_int_robot, force_obstacle, force_total, force_companion;
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
 
  force_total = planner_.get_robot()->get_forces_person_companion(force_to_goal, force_int_person , force_int_robot, force_obstacle, force_companion);

  ros_point_ini.x = planner_.get_robot()->get_current_pointV().x;
  ros_point_ini.y = planner_.get_robot()->get_current_pointV().y;

  if(debug_antes_subgoals_entre_AKP_goals_){
    ROS_INFO("AkpLocalPlanner::markers, ros_point_ini.x=%f=",ros_point_ini.x);
    ROS_INFO("AkpLocalPlanner::markers, ros_point_ini.y=%f=",ros_point_ini.y);

    ROS_INFO("(ROS) AkpLocalPlanner::markers, force_int_person.fx=%f",force_int_person.fx);
    ROS_INFO("(ROS) AkpLocalPlanner::markers, force_int_person.fy=%f",force_int_person.fy);
  }
  //scaled force to goal: ( blue )
  if(this->robot_goal_force_marker)
  {
    if(debug_antes_subgoals_entre_AKP_goals_){
      ROS_INFO("(ROS) robot force goal");
      ROS_INFO("(ROS) ros_point_ini.x=%f",ros_point_ini.x);
      ROS_INFO("(ROS) ros_point_ini.y=%f",ros_point_ini.y);
      ROS_INFO("(ROS) force_to_goal.fx=%f",force_to_goal.fx);
      ROS_INFO("(ROS) force_to_goal.fy=%f",force_to_goal.fy);  
    }

    force_goal_marker_.points[0]= ros_point_ini;
    ros_point.x = ros_point_ini.x + 2*force_to_goal.fx;
    ros_point.y = ros_point_ini.y + 2*force_to_goal.fy;

    if(debug_real_test_companion_){
      ROS_INFO("force_goal_marker_ (ROS) ros_point.x=%f",ros_point.x);
      ROS_INFO("force_goal_marker_ (ROS) ros_point.y=%f",ros_point.y);
    }

    force_goal_marker_.points[1] = ros_point;
    force_goal_marker_.id = cont_f;
    ++cont_f;
    MarkerArray_msg_.markers.push_back(  force_goal_marker_  );
  }

  //scaled person interaction force (green)
  if(this->robot_person_forces_marker)
  { 
    if(debug_antes_subgoals_entre_AKP_goals_){
      ROS_INFO("(ROS) robot force person");
      ROS_INFO("(ROS) force_int_person.fx=%f",force_int_person.fx);
      ROS_INFO("(ROS) force_int_person.fy=%f",force_int_person.fy);
    }

    force_int_person_marker_.points[0]= ros_point_ini;
    ros_point.x = ros_point_ini.x + 2*force_int_person.fx;
    ros_point.y = ros_point_ini.y + 2*force_int_person.fy;
    force_int_person_marker_.points[1] = ros_point;
    force_int_person_marker_.id = cont_f;
    ++cont_f;
    MarkerArray_msg_.markers.push_back(  force_int_person_marker_  );
  }

  //map obstacles interaction force (black)
  if(this->robot_obstacles_forces_marker)
  {
    if(debug_antes_subgoals_entre_AKP_goals_){
      ROS_INFO("(ROS) robot force obstacles");
    }

    force_obstacle_marker_.points[0] = ros_point_ini;
    ros_point.x = ros_point_ini.x + 2*force_obstacle.fx;
    ros_point.y = ros_point_ini.y + 2*force_obstacle.fy;
    force_obstacle_marker_.points[1] =  ros_point;
    force_obstacle_marker_.id = cont_f;
    ++cont_f;
    MarkerArray_msg_.markers.push_back(  force_obstacle_marker_  );
  }

  //weighted resultant force (red)
  if(this->robot_resultant_force_marker)
  {
    if(debug_antes_subgoals_entre_AKP_goals_){
      ROS_INFO("(ROS) robot force resultan");
    }

    force_marker_.points[0] = ros_point_ini;
    ros_point.x = ros_point_ini.x + 2*force_total.fx;
    ros_point.y = ros_point_ini.y + 2*force_total.fy;
    force_marker_.points[1] =  ros_point;
    force_marker_.id = cont_f;
    ++cont_f;
    MarkerArray_msg_.markers.push_back(  force_marker_  );
  }

  //scaled force to goal: ( Cian )
  //if(this->robot_companion_force_marker)
  // {
    if(debug_antes_subgoals_entre_AKP_goals_){
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

  // const Crobot* act_robot=planner_.get_robot();
  Sdestination robot_companion_dest= planner_.get_robot()->get_companion_force_goal();
  //planner_.get_robot()->get_companion_force_goal(robot_companion_dest);
  force_companion_goal_marker_.pose.position.x = robot_companion_dest.x;
  force_companion_goal_marker_.pose.position.y = robot_companion_dest.y;
  force_companion_goal_marker_.id = 20;
  //MarkerArray_msg_.markers.push_back( force_companion_goal_marker_ );
  
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
  //if( goal_providing_mode_ == AkpLocalPlanner::Slicing_global )
 // {
    for( unsigned int i = 0; i< sliced_global_plan_.size(); ++i )
    {
      robot_subgoal_marker_.pose.position.x = sliced_global_plan_[i].pose.position.x;
      robot_subgoal_marker_.pose.position.y = sliced_global_plan_[i].pose.position.y;
      robot_subgoal_marker_.id = cont;
      ++cont;
      MarkerArray_msg_.markers.push_back( robot_subgoal_marker_ );
    }
  //}

  //draw workspace of the robot
  double r = planner_.get_workspace_radii();
	geometry_msgs::Point ros_point, center_point;
  workspace_marker_.points.clear();
  workspace_marker_.id = cont;
  ++cont;
  const std::vector<Spose>* plans = planner_.get_robot()->get_robot_planning_trajectory();

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
*/
    ros_point.x = center_point.x + r*cos( 2*i*PI/40 );
    ros_point.y = center_point.y + r*sin( 2*i*PI/40 );
    //ros_point.x = robot_pose_.x + r*cos( 2*i*PI/40 );
    //ros_point.y = robot_pose_.y + r*sin( 2*i*PI/40 );
    workspace_marker_.points.push_back( ros_point ); 
    ros_point.x = center_point.x + r*cos( (2*i+1)*PI/40 );
    ros_point.y = center_point.y + r*sin( (2*i+1)*PI/40 );
    //ros_point.x = robot_pose_.x + r*cos( 2*i*PI/40 );
    //ros_point.y = robot_pose_.y + r*sin( 2*i*PI/40 );
    workspace_marker_.points.push_back( ros_point ); 
  }
  MarkerArray_msg_.markers.push_back( workspace_marker_ );

  // robot_goal_marker!!!
  Spoint robot_goal=this->planner_.get_robot_goal();
  robot_goal_marker2_.pose.position.x = robot_goal.x;
  robot_goal_marker2_.pose.position.y = robot_goal.y;
  robot_goal_marker2_.id = cont;
  ++cont;
  MarkerArray_msg_.markers.push_back( robot_goal_marker2_ );

  Spoint robot_goal2=this->planner_.get_external_robot_goal();
  robot_goal_marker3_.pose.position.x = robot_goal2.x;
  robot_goal_marker3_.pose.position.y = robot_goal2.y;
  robot_goal_marker3_.id = cont;
  ++cont;
  MarkerArray_msg_.markers.push_back( robot_goal_marker3_ );

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
  
  //std::cout <<"  akp_local_planner_companion!!! "<< std::endl;
  // plot non-dominated solutions in 2d ..................-------------------------------------
  const std::vector<unsigned int>* nondominated_path_index = planner_.get_robot_nondominated_plan_index();
  const std::vector<unsigned int>* nondominated_end_of_path_index = planner_.get_robot_nondominated_end_of_plan_index();
  nd_path2d_marker_.points.clear();
  ROS_INFO("size = %lu" , nondominated_path_index->size());
  ROS_INFO("size = %lu" , nondominated_end_of_path_index->size());

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

void AkpLocalPlanner::fill_planning_markers_3d_companion()
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
  MarkerArray_msg_m2_.markers.push_back( planning_marker_ );

    
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
  MarkerArray_msg_m2_.markers.push_back( best_path_marker_ );
  
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
  MarkerArray_msg_m2_.markers.push_back( nd_path_marker_ );

}






void AkpLocalPlanner::fill_people_prediction_markers_2d()
{
 ROS_INFO( "fill_people_prediction_markers_2d()");
  //plot people trajectories 2d
  const std::vector<unsigned int>* best_path_index = planner_.get_robot_plan_index();
  const std::list<Cperson_abstract *>* person_list = planner_.get_scene( );
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
        //for( unsigned int i = 0; i< best_path_index->size()-less_prediction; ++i , ++cont_2d)
        {
          fill_my_covariance_marker( pred_traj2d_marker_,  traj->at(best_path_index->at(i)), id_person );
          pred_traj2d_marker_.id = cont_2d;
          
            


          MarkerArray_msg_.markers.push_back( pred_traj2d_marker_ );
        }
      }
    //}

  }

}


void AkpLocalPlanner::fill_people_prediction_markers_2d_companion()
{

//ROS_INFO( "fill_people_prediction_markers_2d_companion()");
  //plot people trajectories 2d
  const std::vector<unsigned int>* best_path_index = planner_.get_robot_plan_index();
  const std::list<Cperson_abstract *>* person_list = planner_.get_scene( );
  const std::vector<SpointV_cov>* traj;
  unsigned int cont_2d = 0;
  for( std::list<Cperson_abstract*>::const_iterator iit = person_list->begin();
        iit!=person_list->end(); iit++ )
  {
    traj = (*iit)->get_planning_trajectory();

    unsigned int id_person=(*iit)->get_id();
    //ROS_INFO("id_person= %d", id_person);

    //if((id_person==779)||(id_person==773)||(id_person==775)||(id_person==774)){ //||(id_person==733)
      if (traj->size() > best_path_index->size() )
      //if ( traj->empty() )
      {
        for( unsigned int i = 0; i< best_path_index->size(); ++i , ++cont_2d)
        {
          fill_my_covariance_marker( pred_traj2d_marker_,  traj->at(best_path_index->at(i)), id_person );
          pred_traj2d_marker_.id = cont_2d;

            
          MarkerArray_msg_people_prediction_time_.markers.push_back( pred_traj2d_marker_ );
        }
      }

    //}

  }

}



void AkpLocalPlanner::fill_people_prediction_markers_3d()
{
 ROS_INFO( "fill_people_prediction_markers_3d()");
  const std::list<Cperson_abstract *>* person_list = planner_.get_scene( );
  const std::vector<SpointV_cov>* traj;
  unsigned int cont = 0;
  double time_stamp_ini = planner_.get_time();
  //iter++;

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
    //ROS_INFO("iter= %d", iter);
   // if((id_person==884)){
   
 //||(id_person==733)
      //fill prediction covariances
      for( unsigned int i = 0; i< traj->size(); ++i , ++cont)
     // for( unsigned int i = 0; i< less_prediction; ++i , ++cont)
      {
         //ROS_INFO("i= %d", i);

        fill_my_covariance_marker( pred_traj_marker_,  traj->at(i), id_person);
        pred_traj_marker_.pose.position.z = traj->at(i).time_stamp - time_stamp_ini;//2d cov elevated the time azis(z)
        pred_traj_marker_.id = cont;

            
        MarkerArray_msg_.markers.push_back( pred_traj_marker_ );
      }
   // }

 /* if((iter>135)&&(iter<180)&&(id_person==1011)){
   
 //||(id_person==733)
      //fill prediction covariances
      for( unsigned int i = 0; i< traj->size(); ++i , ++cont)
     // for( unsigned int i = 0; i< less_prediction; ++i , ++cont)
      {
         //ROS_INFO("i= %d", i);
        fill_my_covariance_marker( pred_traj_marker_,  traj->at(i) );
        pred_traj_marker_.pose.position.z = traj->at(i).time_stamp - time_stamp_ini;//2d cov elevated the time azis(z)
        pred_traj_marker_.id = cont;
        MarkerArray_msg_.markers.push_back( pred_traj_marker_ );
      }
    }

if((iter<338)&&(id_person==1019)){
   
 //||(id_person==733)
      //fill prediction covariances
      for( unsigned int i = 0; i< traj->size(); ++i , ++cont)
     // for( unsigned int i = 0; i< less_prediction; ++i , ++cont)
      {
         //ROS_INFO("i= %d", i);
        fill_my_covariance_marker( pred_traj_marker_,  traj->at(i) );
        pred_traj_marker_.pose.position.z = traj->at(i).time_stamp - time_stamp_ini;//2d cov elevated the time azis(z)
        pred_traj_marker_.id = cont;
        MarkerArray_msg_.markers.push_back( pred_traj_marker_ );
      }
    }
if((iter<323)&&(id_person==1023)){
   
 //||(id_person==733)
      //fill prediction covariances
      for( unsigned int i = 0; i< traj->size(); ++i , ++cont)
     // for( unsigned int i = 0; i< less_prediction; ++i , ++cont)
      {
         //ROS_INFO("i= %d", i);
        fill_my_covariance_marker( pred_traj_marker_,  traj->at(i) );
        pred_traj_marker_.pose.position.z = traj->at(i).time_stamp - time_stamp_ini;//2d cov elevated the time azis(z)
        pred_traj_marker_.id = cont;
        MarkerArray_msg_.markers.push_back( pred_traj_marker_ );
      }
    }

  if((iter<300)&&((id_person==1000)||(id_person==1015))){
   
 //||(id_person==733)
      //fill prediction covariances
      for( unsigned int i = 0; i< traj->size(); ++i , ++cont)
     // for( unsigned int i = 0; i< less_prediction; ++i , ++cont)
      {
         //ROS_INFO("i= %d", i);
        fill_my_covariance_marker( pred_traj_marker_,  traj->at(i) );
        pred_traj_marker_.pose.position.z = traj->at(i).time_stamp - time_stamp_ini;//2d cov elevated the time azis(z)
        pred_traj_marker_.id = cont;
        MarkerArray_msg_.markers.push_back( pred_traj_marker_ );
      }
    }*/


  }


}

void AkpLocalPlanner::fill_people_prediction_markers_3d_companion()
{
 ROS_INFO( "fill_people_prediction_markers_3d_companion()");
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

    unsigned int id_person=(*iit)->get_id();
   // ROS_INFO("id_person= %d", id_person);

    //if((id_person==779)||(id_person==773)||(id_person==775)||(id_person==774)){
      //fill prediction covariances
      for( unsigned int i = 0; i< traj->size(); ++i , ++cont)
      {
        
        fill_my_covariance_marker( pred_traj_marker_,  traj->at(i), id_person);
        pred_traj_marker_.pose.position.z = traj->at(i).time_stamp - time_stamp_ini;//2d cov elevated the time azis(z)
        pred_traj_marker_.id = cont;

 
        MarkerArray_msg_people_prediction_time_.markers.push_back( pred_traj_marker_ );
      }
    //}

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


void AkpLocalPlanner::fill_robot_companion_markers()
{
  //ROS_INFO( "entering fill_robot_companion_markers()" );
  //unsigned int cont(0);
  double actual_companion_angle=planner_.get_actual_companion_angle();
   
  // Actuaol robot/trajectory position for this companion angle. (best_next_pose_companion_markers_)
  /*
  //++cont;
  person_companion_marker_.pose.position.x = best_next_pose_companion_markers_.x+1.0*cos(actual_companion_angle*(3.14/180));
  person_companion_marker_.pose.position.y = best_next_pose_companion_markers_.y+1.0*sin(actual_companion_angle*(3.14/180));
  person_companion_marker_.id = 1;
  MarkerArray_msg_.markers.push_back( person_companion_marker_ );
  //ROS_INFO( "person_companion_marker_.pose.position.x=%f", person_companion_marker_.pose.position.x);
  //ROS_INFO( "person_companion_marker_.pose.position.y=%f", person_companion_marker_.pose.position.y);
  //++cont;

 
  robot_companion_marker_.pose.position.x = best_next_pose_companion_markers_.x-1.0*cos(actual_companion_angle*(3.14/180));
  robot_companion_marker_.pose.position.y = best_next_pose_companion_markers_.y-1.0*sin(actual_companion_angle*(3.14/180));
  robot_companion_marker_.id = 2;
  MarkerArray_msg_.markers.push_back( robot_companion_marker_ );
  //ROS_INFO( "robot_companion_marker_.pose.position.x=%f", robot_companion_marker_.pose.position.x);
  //ROS_INFO( "robot_companion_marker_.pose.position.y=%f", robot_companion_marker_.pose.position.y); */
 

    // publish a marcker on the center of robot-person companion.
   center_companion_marker_.pose.position.x = this->robot_pose_.x;
   center_companion_marker_.pose.position.y = this->robot_pose_.y;
   center_companion_marker_.id = 2;
   if(fuera_bolitas_goals_companion_markers_){
      MarkerArray_msg_.markers.push_back( center_companion_marker_ );

    }

  this->target_frame_id = "/map";
  this->source_frame_id = "/tibi/base_footprint";
  std::string source_frame = this->source_frame_id;
  std::string target_frame = this->target_frame_id;
  ros::Time   target_time  = ros::Time::now(); // buscar un mensaje donde se guarte el tiempo actual. o coger el ros time now

  try
  {
    bool tf_exists = tf_listener_.waitForTransform(target_frame, source_frame, target_time, ros::Duration(0.5), ros::Duration(0.01));
//  ROS_INFO("AkpLocalPlanner::fill_robot_companion_markers:  ANTES de  if(tf_exists)");
    if(tf_exists)
    {
      // person transformation
      //ROS_INFO("AkpLocalPlanner::detections_callback:    if(tf_exists)");  
      geometry_msgs::PoseStamped people_poseIn;
      geometry_msgs::PoseStamped people_poseOut;
      people_poseIn.header             = header_point_.header; // guardar un header de un mensaje actual
      people_poseIn.header.frame_id    = source_frame;
      //ROS_INFO("(ROS) AkpLocalPlanner::fill_robot_companion_markers:   actual_companion_angle=%f",actual_companion_angle);  
      people_poseIn.pose.position.x    = -0.75*cos(actual_companion_angle*(3.14/180));
      people_poseIn.pose.position.y    = -0.75*sin(actual_companion_angle*(3.14/180));
      people_poseIn.pose.orientation.z = 1.0;
      tf_listener_.transformPose(target_frame, people_poseIn, people_poseOut);
      person_companion_marker_.pose.position.x = people_poseOut.pose.position.x;
      person_companion_marker_.pose.position.y = people_poseOut.pose.position.y;
      person_companion_marker_.id = 1;
     // MarkerArray_msg_.markers.push_back( person_companion_marker_ );

     //ROS_INFO("AkpLocalPlanner::fill_robot_companion_markers:    if(tf_exists)");  
      geometry_msgs::PoseStamped robot_poseIn;
      geometry_msgs::PoseStamped robot_poseOut;
      robot_poseIn.header             = header_point_.header; // guardar un header de un mensaje actual
      robot_poseIn.header.frame_id    = source_frame;
      robot_poseIn.pose.position.x    = 0.75*cos(actual_companion_angle*(3.14/180));
      robot_poseIn.pose.position.y    = 0.75*sin(actual_companion_angle*(3.14/180));
      robot_poseIn.pose.orientation.z = 1.0;
      tf_listener_.transformPose(target_frame, robot_poseIn, robot_poseOut);
      robot_companion_marker_.pose.position.x = robot_poseOut.pose.position.x;
      robot_companion_marker_.pose.position.y = robot_poseOut.pose.position.y;
      robot_companion_marker_.id = 2;
      //MarkerArray_msg_.markers.push_back( robot_companion_marker_ );

      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(robot_poseOut.pose.position.x, robot_poseOut.pose.position.y, 0.0) );
      //ROS_INFO("(ROS) AkpLocalPlanner::fill_robot_companion_markers:  transform.pose.position.x=%f",transform.pose.position.x);
      //ROS_INFO("(ROS) AkpLocalPlanner::fill_robot_companion_markers: transform.pose.position.y=%f", transform.pose.position.y);    
      transform.setRotation( tf::Quaternion(0, 0, this->robot_pose_.theta) ); // msg->theta= buscar y guardar por arriba orientación robot.
    //  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/tibi/base_footprint"));//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "tibi"));
    }
  }catch (tf::TransformException &ex)
  {
    ROS_INFO("AkpLocalPlanner::fill_robot_companion_markers:: %s",ex.what());
  }
/////////////

  // prueba! ver que goal es el (this->robot_goal_)
  robot_see_goal_marker_.pose.position.x = this->robot_goal_.x;
  robot_see_goal_marker_.pose.position.y = this->robot_goal_.y;
  robot_see_goal_marker_.id = 3;
  //MarkerArray_msg_.markers.push_back( robot_see_goal_marker_ );


}

void AkpLocalPlanner::fill_test_markers(){
  Spoint person=planner_.get_companion_person_position();
  Sdestination person_dest=planner_.get_person_destination();
  double actual_companion_angle=planner_.get_actual_companion_angle();
  double actual_companion_distance=planner_.get_robot_person_proximity_distance();

  double robot_x = this->robot_pose_.x;
  double robot_y = this->robot_pose_.y;
  
  double beta=atan2(robot_y-person.y , robot_x-person.x);

  //ROS_INFO(" [fill_test_markers()] actual_companion_angle=%f",actual_companion_angle);


  double theta =atan2(person_dest.y-person.y , person_dest.x-person.x);
  if( diffangle(theta, beta) < 0 )
  {
    person_90_degre_marker_.pose.position.x=person.x+actual_companion_distance*cos(theta+actual_companion_angle*3.14/180);
    person_90_degre_marker_.pose.position.y=person.y+actual_companion_distance*sin(theta+actual_companion_angle*3.14/180);
  }else{
    person_90_degre_marker_.pose.position.x=person.x+actual_companion_distance*cos(theta-actual_companion_angle*3.14/180);
    person_90_degre_marker_.pose.position.y=person.y+actual_companion_distance*sin(theta-actual_companion_angle*3.14/180);
  }

  person_90_degre_marker_.id = 3;
 // MarkerArray_msg_.markers.push_back( person_90_degre_marker_ );
 // MarkerArray_msg_comp_markers_.markers.push_back( person_90_degre_marker_ );


  // To see the goal to go at X position respect to the accompanied person
  Sdestination robot_goal_from_person_companion=this->planner_.get_person_companion_goal(); 
  robot_goal_to_position_respect_to_the_person.pose.position.x=robot_goal_from_person_companion.x;
  robot_goal_to_position_respect_to_the_person.pose.position.y=robot_goal_from_person_companion.y;
  robot_goal_to_position_respect_to_the_person.id=30;

  if(fuera_bolitas_goals_companion_markers_){
  MarkerArray_msg_.markers.push_back(robot_goal_to_position_respect_to_the_person);
  }
  // to see the goal to go following the path. 
  Sdestination robot_goal_from_planning_traj=this->planner_.get_robot_path_goal();
  robot_goal_to_follow_the_path.pose.position.x=robot_goal_from_planning_traj.x;
  robot_goal_to_follow_the_path.pose.position.y=robot_goal_from_planning_traj.y;
  robot_goal_to_follow_the_path.id=40;

  if(fuera_bolitas_goals_companion_markers_){
  MarkerArray_msg_.markers.push_back(robot_goal_to_follow_the_path);
  }
  // to see the final goal combitation of both and the environment interactions.
  Spose final_robot_goal_pose=this->planner_.get_final_combined_goal();
  final_robot_goal_act_iteration.pose.position.x=final_robot_goal_pose.x;
  final_robot_goal_act_iteration.pose.position.y=final_robot_goal_pose.y;
  final_robot_goal_act_iteration.id=50;
  if(fuera_bolitas_goals_companion_markers_){
  MarkerArray_msg_.markers.push_back(final_robot_goal_act_iteration);
  }
  //if( Action_ROS_==Cplan_local_nav::FACEPERSON){
    Spoint medium_point_face_person=this->planner_.get_medium_point();
    medium_point_face_person_.pose.position.x=final_robot_goal_pose.x;
    medium_point_face_person_.pose.position.y=final_robot_goal_pose.y;
    medium_point_face_person_.id=57;
    if(fuera_bolitas_goals_companion_markers_){
      MarkerArray_msg_.markers.push_back(medium_point_face_person_);
    }
  //}
}


void AkpLocalPlanner::fill_people_prediction_markers_path_points()
{

  const std::list<Cperson_abstract *>* person_list = planner_.get_scene( );
  const std::vector<SpointV_cov>* traj;
  unsigned int cont = 0;
  double time_stamp_ini = planner_.get_time();


  ROS_INFO( "(fill_people_prediction_markers_path_points()) person list %lu" , person_list->size() );
  for( std::list<Cperson_abstract*>::const_iterator iit = person_list->begin();
          iit!=person_list->end(); iit++ )
  {
    if( (*iit)->get_id()==2){ // solo quiero ver la del person target.
        ROS_INFO( "(fill_people_prediction_markers_path_points())  (*iit)->get_id()=%d" ,  (*iit)->get_id() );
      if( (*iit)->get_planning_trajectory()->size() <= 1 )
        traj = (*iit)->get_prediction_trajectory();
      else
        traj = (*iit)->get_planning_trajectory();

      unsigned int id_person=(*iit)->get_id();
     // ROS_INFO("id_person= %d", id_person);

      //if((id_person==779)||(id_person==773)||(id_person==775)||(id_person==774)){
        //fill prediction covariances
        for( unsigned int i = 0; i< traj->size(); ++i , ++cont)
        {
          //fill_my_prediction_points( pred_traj_point_marker_,  traj->at(i) );
          pred_traj_point_marker_.pose.position.x = traj->at(i).x;
          pred_traj_point_marker_.pose.position.y = traj->at(i).y;
          ///marker.pose.position.z = 0.0;

          pred_traj_point_marker_.pose.position.z = 0.0;// traj->at(i).time_stamp - time_stamp_ini;//2d cov elevated the time azis(z)
          pred_traj_point_marker_.id = cont;
          MarkerArray_msg_.markers.push_back( pred_traj_point_marker_ );
        }
      //}

    }
  }
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
    
    // evitar que se pare cuando llega al goal, si luego la persona se dirige a otro goal distinto.
    const std::vector<Sdestination>* dest = planner_.get_destinations();
    Sdestination act_dest;
    if(!dest->empty()){
      act_dest.x=dest->at(0).x;
      act_dest.y=dest->at(0).y;
      this->planner_.get_robot2()->set_best_dest(act_dest);
    }else{
      act_dest.x=0;
      act_dest.y=0;
      this->planner_.get_robot2()->set_best_dest(act_dest);
    }
    for( unsigned int i = 0; i < dest->size(); ++i)
    {
      ROS_INFO("dest->at(0).x=%f ; dest->at(i).y=%f",dest->at(i).x,dest->at(i).y);
    } 
    //doGoal(); // vuelve a lanzar otro goal. (no tiene ni observaciones, entonces, vuelvo a lanzar goal y si la persona esta parada, se para ya de por si!)
	//doGoal2();  // Comprueba antes que el goal sea accesible al ROBOT. vuelve a lanzar otro goal. (no tiene ni observaciones, entonces, vuelvo a lanzar goal y si la persona esta parada, se para ya de por si!)

 /*     ROS_INFO("obs.size()=%f",obs.size());
    for(unsigned int i=0 ; i< obs.size() ; ++i)
	  {
      if(obs[i].id==id_person_companion_){ // si la velocidad de la persona a la que acompaño es diferente de 0. Vuelve a lanzar un goal!.
             ROS_INFO("person id=%f",id_person_companion_);
       if(((obs[i].vx*obs[i].vx)>0)||((obs[i].vy*obs[i].vy)>0)){
            ROS_INFO("justo antes doGoal");
            doGoal(); // vuelve a lanzar otro goal.
        }
      }
    }*/
    //return 1;
    //this->good_goal_=true;
  }
  else
  {
    ROS_WARN("moveRobot: Oops, the base failed to reach the goal for some reason!");
    //return 2;
    //this->good_goal_=false;  
  } 

  this->isMoveBaseActive=false;
  //this->alg_.unlock(); 
} 

void AkpLocalPlanner::move_baseActive() 
{ 
  //this->alg_.lock(); 
  //ROS_INFO("AkpLocalPlanner::move_baseActive: Goal just went active!"); 
  

  //this->alg_.unlock(); 
} 

void AkpLocalPlanner::move_baseFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) 
{ 
  //ROS_INFO(" IN move_baseFeedback ");
  //this->alg_.lock(); 
  bool feedback_is_ok = true; 
  if( !feedback_is_ok ) 
    this->move_base_client_.cancelGoal(); 


//ROS_INFO(" [IN move_baseFeedback] feedback_is_ok=%f ",feedback_is_ok);
  //this->alg_.unlock(); 
}


/////////////  [action requests] //////////////////
bool AkpLocalPlanner::move_baseMakeActionRequest() 
{ 
  //ROS_INFO("[SENDING GOAL] move_baseMakeActionRequest()");
  bool ok=false;
  //if(this->config.move_base)
  //{
    if(this->move_base_client_.isServerConnected())
    {
      //this->alg_.unlock();
      this->move_base_client_.sendGoal(move_base_goal_, 
                  boost::bind(&AkpLocalPlanner::move_baseDone,     this, _1, _2), 
                  boost::bind(&AkpLocalPlanner::move_baseActive,   this), 
                  boost::bind(&AkpLocalPlanner::move_baseFeedback, this, _1));
      //this->alg_.lock();
      //ROS_INFO("AkpLocalPlanner::move_baseMakeActionRequest: Goal Sent!");
      this->isMoveBaseActive=true;
      ok=true;
      
      //ROS_INFO("Goal sent. Waiting for result...");
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

bool AkpLocalPlanner::doGoal() //(std::vector<double> goal) bool AkpLocalPlanner::doGoal(Spoint goal)
{
  //this->getAndPublishImage(); //AG150730: assure that the image is published
  //if((this->debug2)||(this->config_.debug)){
  //ROS_INFO("AkpLocalPlanner::doGoal: Goal is x=%f, y=%f",goal.x,goal.y);
  //}

  Sdestination best_person_dest=this->planner_.get_robot2()->get_best_dest();
  
  Spose actual_robot=this->planner_.get_robot2()->get_current_pose();


  this->move_base_goal_.target_pose.pose.position.x = best_person_dest.x; //goal[0];
  this->move_base_goal_.target_pose.pose.position.y = best_person_dest.y; //goal[1];
  double theta = atan2(best_person_dest.y - this->robot_pose_.y , best_person_dest.x - this->robot_pose_.x);
  this->move_base_goal_.target_pose.pose.orientation.x = 0;//goal.get_robot_orient_x();
  this->move_base_goal_.target_pose.pose.orientation.y = 0;//goal.get_robot_orient_y();
  if(theta<0.002){ // To avoid error:  Quaternion has length close to zero... discarding as navigation goal
    theta=0.002;
  }

	this->move_base_goal_.target_pose.pose.orientation.z = theta;//goal.get_robot_orient_z();
	this->move_base_goal_.target_pose.pose.orientation.w = 0;//goal.get_robot_orient_w();
  

  this->move_base_goal_.target_pose.header.stamp    = ros::Time::now();
  this->move_base_goal_.target_pose.header.frame_id = "map";// == desde que frame_id mandas el goal, cambiar si va mal!. this->config.faf_frame; (lo que habia antes)
//ROS_INFO("OUTTT AkpLocalPlanner::doGoal:");

 //ROS_INFO("AkpLocalPlanner::doGoal: robot position x=%f, y=%f",actual_robot.x,actual_robot.y);
 //ROS_INFO("AkpLocalPlanner::doGoal: Goal is x=%f, y=%f; theta=%f",best_person_dest.x,best_person_dest.y,theta);


   //ROS_INFO("IMPORTANT!!!! IMPORTANT!!! actual case true!!!  x=%f, y=%f theta=%f!",this->move_base_goal_.target_pose.pose.position.x,this->move_base_goal_.target_pose.pose.position.y,this->move_base_goal_.target_pose.pose.orientation.z); 
  

    next_goal_marker_.pose.position.x = best_person_dest.x;
    next_goal_marker_.pose.position.y = best_person_dest.y;
	next_goal_marker_.pose.orientation.z = theta;
    next_goal_marker_.id = 17;
    
    if(fuera_bolitas_goals_companion_markers_){
      MarkerArray_msg_.markers.push_back( next_goal_marker_ ); 
    }

  if(this->move_baseMakeActionRequest())
    return true;
  else
    return false;



} 

bool AkpLocalPlanner::doGoal2() //(std::vector<double> goal) bool AkpLocalPlanner::doGoal(Spoint goal)
{

  if(get_scout_results_doGoal_){
    ROS_INFO("AkpLocalPlanner::doGoal2: start");
  }

  Sdestination best_person_dest=this->planner_.get_robot2()->get_best_dest();
  
  Spose actual_robot=this->planner_.get_robot2()->get_current_pose();

  geometry_msgs::PoseStamped start_position;
  //start_position.header             = header_point_.header; // guardar un header de un mensaje actual
  start_position.header.stamp    = ros::Time::now();
  start_position.header.frame_id    =  "map";
  start_position.pose.position.x    = this->robot_pose_.x;
  start_position.pose.position.y    = this->robot_pose_.y;

	

  if(this->robot_pose_.theta<0.002){ // To avoid error:  Quaternion has length close to zero... discarding as navigation goal
    this->robot_pose_.theta=0.002;
  }
  tf::Quaternion robot_quat = tf::createQuaternionFromYaw(this->robot_pose_.theta);
  start_position.pose.orientation.x = robot_quat.x();
  start_position.pose.orientation.y = robot_quat.y();
  start_position.pose.orientation.z = robot_quat.z();
  start_position.pose.orientation.w = robot_quat.w();
  //start_position.pose.orientation.z = this->robot_pose_.theta;

  geometry_msgs::PoseStamped goal_position;
 // goal_position.header             = header_point_.header; // guardar un header de un mensaje actual
  goal_position.header.stamp    = ros::Time::now();
  goal_position.header.frame_id    =  "map";
  goal_position.pose.position.x    = best_person_dest.x;
  goal_position.pose.position.y    = best_person_dest.y;
  double theta_goal = atan2(best_person_dest.y - this->robot_pose_.y , best_person_dest.x - this->robot_pose_.x);
  if(theta_goal<0.002){ // To avoid error:  Quaternion has length close to zero... discarding as navigation goal
    theta_goal=0.002;
  }

  //theta_goal = 0.5;
  tf::Quaternion goal_quat = tf::createQuaternionFromYaw(theta_goal);
  goal_position.pose.orientation.x = goal_quat.x();
  goal_position.pose.orientation.y = goal_quat.y();
  goal_position.pose.orientation.z = goal_quat.z();
  goal_position.pose.orientation.w = goal_quat.w();
 
 
  std::vector<geometry_msgs::PoseStamped> Generated_plan;


  get_plan_srv_.request.start = start_position;
  get_plan_srv_.request.goal = goal_position;
  get_plan_srv_.request.tolerance = 1.5;

  if(get_scout_results_doGoal_){
    ROS_INFO("AkpLocalPlanner::doGoal2: get_plan_srv_.request.tolerance= %f",get_plan_srv_.request.tolerance);
    ROS_INFO("AkpLocalPlanner::doGoal2: start x= %f ; y=%f",get_plan_srv_.request.start.pose.position.x, get_plan_srv_.request.start.pose.position.y );
    ROS_INFO("AkpLocalPlanner::doGoal2: goal x= %f ; y=%f",get_plan_srv_.request.goal.pose.position.x, get_plan_srv_.request.goal.pose.position.y );
  }

  bool good_plan=false;
  bool call_succed=get_plan_client_.call(get_plan_srv_); 

  if(get_scout_results_doGoal_){
    if(call_succed){
      ROS_INFO("call_succed=true"); 
    }else{
      ROS_INFO("IMPORTANT!!!! IMPORTANT!!! call_succed=false"); 
    }
  }
  nav_msgs::Path new_generated_path;

  //if(call_succed){
     new_generated_path=get_plan_srv_.response.plan;
  //}

  if(get_scout_results_doGoal_){
    ROS_INFO("AkpLocalPlanner::doGoal: SIZE new_path=%lu",new_generated_path.poses.size());
    ROS_INFO("AkpLocalPlanner::doGoal: SIZE et_plan_srv_.response.plan=%lu",get_plan_srv_.response.plan.poses.size());
    long unsigned int poses_size = new_generated_path.poses.size();
    long unsigned int poses_size2 = sizeof(get_plan_srv_.response.plan.poses.size());
    ROS_INFO("AkpLocalPlanner::doGoal: 2 SIZE new_path=%lu",poses_size);
    ROS_INFO("AkpLocalPlanner::doGoal: 2 SIZE et_plan_srv_.response.plan=%lu",poses_size2);
  }

  if(new_generated_path.poses.empty()){
    good_plan=false;
    if(get_scout_results_doGoal_){
      ROS_INFO("IMPORTANT!!!! IMPORTANT!!! good_plan=falsee (empty poses) Not plan!"); 
    }
  }else{
    good_plan=true;
    if(get_scout_results_doGoal_){
      ROS_INFO("AkpLocalPlanner::doGoal: new_generated_path.poses.empty()=false ");
    }
  }


  if(get_scout_results_doGoal_){
    ROS_INFO("AkpLocalPlanner::doGoal: robot position x=%f, y=%f",this->robot_pose_.x,this->robot_pose_.y);
    ROS_INFO("AkpLocalPlanner::doGoal: robot position x=%f, y=%f",actual_robot.x,actual_robot.y);
    ROS_INFO("AkpLocalPlanner::doGoal: Goal is x=%f, y=%f; theta=%f",best_person_dest.x,best_person_dest.y,theta_goal);
   if(good_plan){ 
      ROS_INFO("good_plan=true"); 
    }else{ 
       ROS_INFO("good_plan=false"); 
    }
}

  if(good_plan){

    this->move_base_goal_.target_pose.pose.position.x = best_person_dest.x; //goal[0];
    this->move_base_goal_.target_pose.pose.position.y = best_person_dest.y; //goal[1];

    double theta = atan2(best_person_dest.y - this->robot_pose_.y , best_person_dest.x - this->robot_pose_.x);
    this->move_base_goal_.target_pose.pose.orientation.x = 0;//goal.get_robot_orient_x();
    this->move_base_goal_.target_pose.pose.orientation.y = 0;//goal.get_robot_orient_y();
    if(theta<0.002){ // To avoid error:  Quaternion has length close to zero... discarding as navigation goal
      theta=0.002;
    }

    this->move_base_goal_.target_pose.pose.orientation.z = theta;//goal.get_robot_orient_z();
    this->move_base_goal_.target_pose.pose.orientation.w = 0;//goal.get_robot_orient_w();
    //ROS_INFO("Next robot goal: (%f,%f) %fº",goal[0],goal[1],goal[2]*180.0/M_PI); 

    this->move_base_goal_.target_pose.header.stamp    = ros::Time::now();
    this->move_base_goal_.target_pose.header.frame_id = "map";// == desde que frame_id mandas el goal, cambiar si va mal!. this->config.faf_frame; (lo que habia antes)
    
    this->before_good_move_base_goal_=this->move_base_goal_;

    //if(get_scout_results_doGoal_){
     // ROS_INFO("IMPORTANT!!!! IMPORTANT!!! actual case true!!!  x=%f, y=%f theta=%f!",this->move_base_goal_.target_pose.pose.position.x,this->move_base_goal_.target_pose.pose.position.y,this->move_base_goal_.target_pose.pose.orientation.z); 
      //ROS_INFO("IMPORTANT!!!! IMPORTANT!!! actual case true!!!  x=%f, y=%f!",this->move_base_goal_.target_pose.pose.position.x,this->move_base_goal_.target_pose.pose.position.y); 
    //}

    next_goal_marker_.pose.position.x = best_person_dest.x;
    next_goal_marker_.pose.position.y = best_person_dest.y;
 	next_goal_marker_.pose.orientation.z = theta;
    next_goal_marker_.id = 17;
    
    if(fuera_bolitas_goals_companion_markers_){
      MarkerArray_msg_.markers.push_back( next_goal_marker_ ); 
    }

  }else{

    this->move_base_goal_=this->before_good_move_base_goal_;
    next_goal_marker_.pose.position.x = before_good_move_base_goal_.target_pose.pose.position.x;
    next_goal_marker_.pose.position.y = before_good_move_base_goal_.target_pose.pose.position.y;
    next_goal_marker_.id = 17;
    
    if(fuera_bolitas_goals_companion_markers_){
      MarkerArray_msg_.markers.push_back( next_goal_marker_ ); 
    }


    if(get_scout_results_doGoal_){
      ROS_INFO("IMPORTANT!!!! IMPORTANT!!! else use before good goal!!!  x=%f, y=%f!",this->before_good_move_base_goal_.target_pose.pose.position.x,this->before_good_move_base_goal_.target_pose.pose.position.y); 
    }
  }

 
  if(this->move_baseMakeActionRequest())
    return true;
  else
    return false;



} 




/*  subscript to PS3 and wii (comandaments) to set up or down the beta weigth of the forces formula for learning alpha and beta of the companion task */

void AkpLocalPlanner::useWii(std::vector<int> current_buttons, std::vector<int> last_buttons)
{
  if(current_buttons[wiimote::State::MSG_BTN_B]==0)
  {
    this->human_is_alive_=false;
   // this->twist_msg.linear.x=0.0;
   // this->twist_msg.angular.z=0.0;
    if(last_buttons[wiimote::State::MSG_BTN_B]==1)
    {
      ROS_INFO("Released dead man button. Stop!");
     // this->cmd_vel_publisher_.publish(this->twist_msg); // I think that is only needed for the teleop, not needed in my case.
    }
    for(unsigned int i=0; i<current_buttons.size(); i++) // solo coges el boton si NO esta pulsado el dead_man_buton!
    {
      if(current_buttons[i]==1 && last_buttons[i]==0)
        this->useWiiButton(i);
    }
  }
  else
  {
    this->human_is_alive_=true;

  }
}

void AkpLocalPlanner::useWiiButton(const unsigned int & index)
{
  //double vt_step = this->config_.translation_increment_step;  // needed!?
  //double vr_step = this->config_.rotation_increment_step;  // needed!?
  double act_beta=planner_.get_beta_companion();

  int case_Up=wiimote::State::MSG_BTN_PLUS;
  int case_Down=wiimote::State::MSG_BTN_MINUS;
  if(!use_default_wii_button_){
    case_Up=config_.Wii_Up_button;
    case_Down=config_.Wii_Down_button;
  }


  if(index==case_Up){
    ROS_INFO("UP Beta!");
    // double act_beta=planner_.get_beta_companion();
    act_beta=act_beta+0.1;
    planner_.set_beta_companion(act_beta);
    planner_.set_alpha_companion(1-act_beta);
  }

  if(index==case_Down){
    ROS_INFO("DOWN Beta!");
    //double act_beta=planner_.get_beta_companion();
    act_beta=act_beta-0.1;
    planner_.set_beta_companion(act_beta);
    planner_.set_alpha_companion(1-act_beta);
  }

  /* switch(index)
 {
      // opcion 1, mando wii
      case case_Up: // case Beta up
        ROS_INFO("UP Beta!");
       // double act_beta=planner_.get_beta_companion();
        act_beta=act_beta+0.1;
        planner_.set_beta_companion(act_beta);
        planner_.set_alpha_companion(1-act_beta);

      break;

      case case_Down: // case Beta down
        ROS_INFO("DOWN Beta!");
        //double act_beta=planner_.get_beta_companion();
        act_beta=act_beta-0.1;
        planner_.set_beta_companion(act_beta);
        planner_.set_alpha_companion(1-act_beta);

      break;*/

      // opcion 2, mando wii
 /*     case wiimote::State::MSG_BTN_1: // case Beta up
        ROS_INFO("UP Beta!");
        //double act_beta=planner_.get_beta_companion();
        act_beta=act_beta+0.1;
        planner_.set_beta_companion(act_beta);
        planner_.set_alpha_companion(1-act_beta);

      break;

      case wiimote::State::MSG_BTN_2: // case Beta down
        ROS_INFO("DOWN Beta!");
        //double act_beta=planner_.get_beta_companion();
        act_beta=act_beta-0.1;
        planner_.set_beta_companion(act_beta);
        planner_.set_alpha_companion(1-act_beta);

      break;

      // opcion 3, mando wii
      case wiimote::State::MSG_BTN_UP:
        ROS_INFO("UP Beta!");

        //double act_beta=planner_.get_beta_companion();
        act_beta=act_beta+0.1;
        planner_.set_beta_companion(act_beta);
        planner_.set_alpha_companion(1-act_beta);

      break;

      case wiimote::State::MSG_BTN_DOWN:
        ROS_INFO("DOWN Beta!");

        //double act_beta=planner_.get_beta_companion();
        act_beta=act_beta-0.1;
        planner_.set_beta_companion(act_beta);
        planner_.set_alpha_companion(1-act_beta);

      break;*/

   /* // these cases comented, we do not use it, only used for the teleop.
     case wiimote::State::MSG_BTN_B:
      //already considered (dead_man button)
      break;

    case wiimote::State::MSG_BTN_A:
      //ROS_INFO("A:stop!");
      this->twist_msg.linear.x=0.0;
      this->twist_msg.angular.z=0.0;
      ROS_INFO("Pressed A button. Stop!");
      this->cmd_vel_publisher_.publish(this->twist_msg);
      if(this->cancel_goal)
      {
        // Uncomment the following line to publish the topic message
        this->cancel_goal_publisher_.publish(this->cancel_goal_GoalID_msg_);
      }
      break;

    case wiimote::State::MSG_BTN_LEFT:
      //ROS_INFO("left!");
      this->twist_msg.angular.z += vr_step;
      if(this->twist_msg.angular.z > this->vr_max)
        this->twist_msg.angular.z = this->vr_max;
      break;

    case wiimote::State::MSG_BTN_RIGHT:
      //ROS_INFO("right!");
      this->twist_msg.angular.z -= vr_step;
      if(this->twist_msg.angular.z < -this->vr_max)
        this->twist_msg.angular.z = -this->vr_max;
      break;

    case wiimote::State::MSG_BTN_UP:
      //ROS_INFO("forward!");
      this->twist_msg.linear.x += vt_step;
      if(this->twist_msg.linear.x > this->vt_max)
        this->twist_msg.linear.x = this->vt_max;
      break;

    case wiimote::State::MSG_BTN_DOWN:
      //ROS_INFO("backward!");
      this->twist_msg.linear.x -= vt_step;
      if(this->twist_msg.linear.x < -this->vt_max)
        this->twist_msg.linear.x = -this->vt_max;
      break;*/

  //  default:
      //ROS_INFO("non defined button");
 //     break;
 // }
}

void AkpLocalPlanner::usePs3(std::vector<int> current_buttons, std::vector<int> last_buttons, std::vector<float> current_axes)
{
  if(current_buttons[BUTTON_DEAD_MAN]==0)
  {
   this->human_is_alive_=false;
   // this->twist_msg.linear.x=0.0;
   // this->twist_msg.angular.z=0.0;
    if(last_buttons[BUTTON_DEAD_MAN]==1)
    {
      ROS_INFO("Released dead man button. Stop!");
     // this->cmd_vel_publisher_.publish(this->twist_msg); // I think that is only needed for the teleop, not needed in my case.
    //  this->trans_speed_scale=this->config_.translation_increment_step; // needed!?
    //  this->rot_speed_scale=this->config_.rotation_increment_step; // needed!?
    }

    for(unsigned int i=0; i<current_buttons.size(); i++) // solo coges el boton si NO esta pulsado el dead_man_buton!
    {
      if(current_buttons[i]==1 && last_buttons[i]==0)
        this->usePs3Button(i);
    }
  }
  else
  {
    this->human_is_alive_=true;

   // double vt = this->trans_speed_scale*this->vt_max*current_axes[AXIS_TRANS_FORWARD]; // needed!?
   // double vr = this->rot_speed_scale*this->vr_max*current_axes[AXIS_ROT_LEFTWARD]; // needed!?
   // this->twist_msg.linear.x  = vt;  // needed!?
   // this->twist_msg.angular.z = vr;  // needed!?
  }
}

void AkpLocalPlanner::usePs3Button(const unsigned int & index)
{

  double act_beta=planner_.get_beta_companion();

  int case_Up=BETA_UP;
  int case_Down=BETA_DOWN;
  int case_select_near_pers_as_companion=NEAR_PERS_COMP;
  if(!use_default_PS3_button_){
    case_Up=config_.PS3_Up_button;
    case_Down=config_.PS3_Down_button;
    //case_select_near_pers_as_companion=config_.
  }


  if(index==case_Up){
    ROS_INFO("UP Beta!");
    // double act_beta=planner_.get_beta_companion();
    act_beta=act_beta+0.1;
    planner_.set_beta_companion(act_beta);
    planner_.set_alpha_companion(1-act_beta);
  }

  if(index==case_Down){
    ROS_INFO("DOWN Beta!");
    //double act_beta=planner_.get_beta_companion();
    act_beta=act_beta-0.1;
    planner_.set_beta_companion(act_beta);
    planner_.set_alpha_companion(1-act_beta);
  }

  if(index==case_select_near_pers_as_companion){
  
    Spoint actual_robot_pose;   //observation[i].id 
    actual_robot_pose.x=this->robot_pose_.x;
    actual_robot_pose.y=this->robot_pose_.y;
    actual_robot_pose.time_stamp=this->robot_pose_.time_stamp;

    double min_distance;
    unsigned int id_min_dist_per;
    for(unsigned int t=0; t<this->obs.size(); t++){
	Spoint actual_person;
	actual_person.x=this->obs[t].x;
	actual_person.y=this->obs[t].y;
	actual_person.time_stamp=this->obs[t].time_stamp;
	
	if(t==0){
	  min_distance=actual_person.distance(actual_robot_pose);
	  id_min_dist_per=this->obs[t].id;
	}
	if(actual_person.distance(actual_robot_pose)<min_distance){
	  min_distance=actual_person.distance(actual_robot_pose);
          id_min_dist_per=this->obs[t].id;
	}
    }
    this->planner_.set_id_person_companion(id_min_dist_per);
    id_person_companion_=id_min_dist_per;
    //this->config.id_person_companion=id_min_dist_per;
    flag_play_change_id_=true;
    ROS_INFO("case select near person as companion! id_person=%d",id_min_dist_per);
   
  }



/*  switch(index)
  {
      case case_Up: // case Beta up
        ROS_INFO("UP Beta!");
       // double act_beta=planner_.get_beta_companion();
        act_beta=act_beta+0.1;
        planner_.set_beta_companion(act_beta);
        planner_.set_alpha_companion(1-act_beta);

      break;

      case case_Down: // case Beta down

        ROS_INFO("DOWN Beta!");
        //double act_beta=planner_.get_beta_companion();
        act_beta=act_beta-0.1;
        planner_.set_beta_companion(act_beta);
        planner_.set_alpha_companion(1-act_beta);

      break;*/

   /* // these cases comented, we do not use it, only used for the teleop.
     case BUTTON_DEAD_MAN:
      //already considered
      break;

    case BUTTON_TRANS_SPEED_UP:
      this->trans_speed_scale+=this->config_.translation_increment_step;
      if(this->trans_speed_scale>1.0)
        this->trans_speed_scale=1.0;
      else
        ROS_INFO("Setting max translation speed to %.2f (%.0f%% of max %.2f)",
                 this->trans_speed_scale*this->vt_max,
                 this->trans_speed_scale*100.0,
                 this->vt_max);
      break;

    case BUTTON_TRANS_SPEED_DOWN:
      this->trans_speed_scale-=this->config_.translation_increment_step;
      if(this->trans_speed_scale<0.0)
        this->trans_speed_scale=0.0;
      else
        ROS_INFO("Setting max translation speed to %.2f (%.0f%% of max %.2f)",
                 this->trans_speed_scale*this->vt_max,
                 this->trans_speed_scale*100.0,
                 this->vt_max);
      break;

    case BUTTON_ROT_SPEED_UP:
      this->rot_speed_scale+=this->config_.rotation_increment_step;
      if(this->rot_speed_scale>1.0)
        this->rot_speed_scale=1.0;
      else
        ROS_INFO("Setting max rotation speed to %.2f (%.0f%% of max %.2f)",
                 this->rot_speed_scale*this->vr_max,
                 this->rot_speed_scale*100.0,
                 this->vr_max);
      break;

    case BUTTON_ROT_SPEED_DOWN:
      this->rot_speed_scale-=this->config_.rotation_increment_step;
      if(this->rot_speed_scale<0.0)
        this->rot_speed_scale=0.0;
      else
        ROS_INFO("Setting max rotation speed to %.2f (%.0f%% of max %.2f)",
                 this->rot_speed_scale*this->vr_max,
                 this->rot_speed_scale*100.0,
                 this->vr_max);
      break;

    case BUTTON_CANCEL_GOAL: 
      if(this->cancel_goal)
      {
        // Uncomment the following line to publish the topic message
        this->cancel_goal_publisher_.publish(this->cancel_goal_GoalID_msg_);
      }
      break;*/
  //  default:
      //ROS_INFO("non defined button");
   //   break;
  //}
}

void AkpLocalPlanner::reset_joy_watchdog(void)
{
  this->joy_watchdog_access.enter();
  this->joy_watchdog_duration=ros::Duration(this->config_.joy_watchdog_time);
  this->joy_watchdog_access.exit();
}

bool AkpLocalPlanner::joy_watchdog_active(void)
{
  this->joy_watchdog_access.enter();
  if(this->joy_watchdog_duration.toSec()<=0.0)
  {
    this->joy_watchdog_access.exit();
    return true;
  }
  else
  {
    this->joy_watchdog_access.exit();
    return false;
  }
}

void AkpLocalPlanner::update_joy_watchdog(void)
{
  static ros::Time start_time=ros::Time::now();
  ros::Time current_time=ros::Time::now();

  this->joy_watchdog_access.enter();
  this->joy_watchdog_duration-=(current_time-start_time);
  start_time=current_time;
  this->joy_watchdog_access.exit();
}



void AkpLocalPlanner::function_joy_teleop_mainNodeThread(void) // todo, mirar si es necesaria que creo que no...
{
  this->alg_.lock();
  this->update_joy_watchdog();
  if(this->human_is_alive_)
  {
    if(this->joy_watchdog_active())
    {
      this->human_is_alive_ = false;
      ROS_ERROR("PlatformTeleopAlgNode::mainNodeThread: joy_watchdog timeout!");
     // this->twist_msg.linear.x=0.0;
     // this->twist_msg.angular.z=0.0;
      //this->cmd_vel_publisher_.publish(this->twist_msg);
     // this->trans_speed_scale=this->config_.translation_increment_step;
     // this->rot_speed_scale=this->config_.rotation_increment_step;
    }
    else{
     // this->cmd_vel_publisher_.publish(this->twist_msg);
    }
  }
  this->alg_.unlock();

  // [fill msg structures]
  // Initialize the topic message structure
  //this->cancel_goal_GoalID_msg_.data = my_var;


  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]


}


// to stop the code, to evaluate the costs

void AkpLocalPlanner::cmd_vel_stop_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  if(debug_stop_node_to_evaluate_costs_){
    ROS_INFO("AkpLocalPlanner::cmd_vel_stop_callback: New Message Received");
  }
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
    std::cout <<"linear_x="<<linear_x<<std::endl;
    std::cout <<"linear_y="<<linear_y<<std::endl;
    std::cout <<"linear_z="<<linear_z<<std::endl;
    std::cout <<"angular_x="<<angular_x<<std::endl;
    std::cout <<"angular_y="<<angular_y<<std::endl;
    std::cout <<"angular_z="<<angular_z<<std::endl;
  }

  if((linear_x==7)&&(angular_z==7)){
     if(debug_stop_node_to_evaluate_costs_){
       ROS_INFO("NEW ITERATION");
       std::cout <<std::endl<<std::endl<<std::endl;
     }
      //this->start=true;
      this->current_state=HSQ_IT;
  }

  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->cmd_vel_stop_mutex_exit();
}

