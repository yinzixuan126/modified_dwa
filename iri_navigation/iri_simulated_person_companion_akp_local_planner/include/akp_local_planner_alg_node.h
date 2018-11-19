// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _akp_local_planner_alg_node_h_
#define _akp_local_planner_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "akp_local_planner_alg.h"
#include "scene_sim.h"

#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>

#include <dynamic_reconfigure/server.h>
#include <iri_simulated_person_companion_akp_local_planner/AkpLocalPlannerConfig.h>
 

// [publisher subscriber headers]
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <iri_perception_msgs/detectionArray.h>
#include <iri_perception_msgs/restartSim.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h> // robot companion (ely) trasladar modelo tibi robot companion al marcker donde deberia estar el robot.

// [service client headers]

// [action server client headers]
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>

// [libraries]
#include "nav/plan_local_nav.h"


// Cscene_sim

// [service client headers]
#include <std_srvs/Empty.h>
#include <iri_perception_msgs/InitialiceSim.h>

typedef enum {
  HSQ_INIT,
  HSQ_IT,
  HSQ_STOP,
  HSQ_RECEIVE_OBS,
  HSQ_SEND_GOAL
} states;



/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class AkpLocalPlanner : public nav_core::BaseLocalPlanner
{
  enum Goal_providing_mode { Crop_local_window=0, Slicing_global};
  private:

/***  !!!CLAS planning Atributes!!!  (ini) ***/
    // [publisher attributes]
    ros::Publisher cost_params_publisher_;
    std_msgs::Float64MultiArray Float64_msg_;
    
    ros::Publisher cmd_vel_publisher_;
    geometry_msgs::Twist cmd_vel_msg_;

    ros::Publisher g_plan_pub_;
    ros::Publisher l_plan_pub_;

    pthread_mutex_t planner_mutex_;
    void planner_mutex_enter(void);
    void planner_mutex_exit(void);

    ros::Publisher markers_publisher_;
    ros::Publisher markers_publisher_m2_;
    ros::Publisher markers_publisher_comp_markers_;
    ros::Publisher markers_publisher_people_prediction_time_;

   // visualization_msgs::MarkerArray MarkerArray_msg_;
   // visualization_msgs::MarkerArray MarkerArray_msg_comp_markers_;
   // visualization_msgs::MarkerArray MarkerArray_msg_m2_;
   // visualization_msgs::MarkerArray MarkerArray_msg_people_prediction_time_;

    visualization_msgs::Marker pred_traj_marker_, 
      pred_traj2d_marker_,
      cylinder_marker_,
      planning_marker_,
      robot_goal_marker_,
      robot_subgoal_marker_,
      robot_marker_,
      workspace_marker_,
      planning_goals_marker_,
      best_path_marker_,
      best_path2d_marker_,
      laser_obstacle_marker_,
      force_marker_,
      force_goal_marker_,
      force_int_person_marker_,
      force_obstacle_marker_,
      force_int_robot_marker_,
      text_marker_,
      nd_path_marker_,
      nd_path2d_marker_,
      person_companion_marker_,
      robot_companion_marker_,
      center_companion_marker_,
      robot_see_goal_marker_,
      person_90_degre_marker_,
      force_companion_marker_,
      force_companion_goal_marker_,
      best_next_pose_robot_companion_markers_,
      robot_goal_marker2_,
      robot_goal_marker3_,
      goal_path_marker_,
      goal_companion_marker_;

    // [subscriber attributes]
    ros::Subscriber fscan_subscriber_;
    void fscan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    pthread_mutex_t fscan_mutex_;
    void fscan_mutex_enter(void);
    void fscan_mutex_exit(void);
    sensor_msgs::LaserScan fscan;
    bool fscan_received;

    ros::Subscriber rscan_subscriber_;
    void rscan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    pthread_mutex_t rscan_mutex_;
    void rscan_mutex_enter(void);
    void rscan_mutex_exit(void);
    sensor_msgs::LaserScan rscan;
    bool rscan_received;

    ros::Subscriber odom_subscriber_;
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    pthread_mutex_t odom_mutex_;
    void odom_mutex_enter(void);
    void odom_mutex_exit(void);

    ros::Subscriber odom_subscriber_tibi_;
    void odom_callback_tibi_robot(const nav_msgs::Odometry::ConstPtr& msg);   
    pthread_mutex_t odom_tibi_mutex_;
    void odom_tibi_mutex_enter(void);
    void odom_tibi_mutex_exit(void);

    ros::Subscriber odom_subscriber_other_person_comp_of_group_;
    void odom_callback_other_person_comp_of_group(const nav_msgs::Odometry::ConstPtr& msg); 
    pthread_mutex_t odom_other_person_comp_of_group_mutex_;
    void odom_other_person_comp_of_group_mutex_enter(void);
    void odom_other_person_comp_of_group_mutex_exit(void);

    ros::Subscriber tracks_subscriber_;
    void tracks_callback(const iri_perception_msgs::detectionArray::ConstPtr& msg);
    pthread_mutex_t tracks_mutex_;
    void tracks_mutex_enter(void);
    void tracks_mutex_exit(void);
    
    //only necessary for learning purposes
    ros::Subscriber params_values_subscriber_;
    void params_values_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    pthread_mutex_t params_values_mutex_;
    void params_values_mutex_enter(void);
    void params_values_mutex_exit(void);


// for group of 3 people implementation:
    ros::Subscriber other_person_companion_tracks_subscriber_;
    void other_person_companion_tracks_callback(const iri_perception_msgs::detectionArray::ConstPtr& msg);
    pthread_mutex_t other_person_companion_tracks_mutex_;
    void other_person_companion_tracks_mutex_enter(void);
    void other_person_companion_tracks_mutex_exit(void);

    ros::Subscriber other_person_destination_tracks_subscriber_;
    void other_person_destination_tracks_callback(const iri_perception_msgs::detectionArray::ConstPtr& msg);
    pthread_mutex_t other_person_destination_tracks_mutex_;
    void other_person_destination_tracks_mutex_enter(void);
    void other_person_destination_tracks_mutex_exit(void);


  /* ros::Subscriber status_init_subscriber_;
    void status_init_callback(const iri_perception_msgs::restartSim::ConstPtr& msg);
    pthread_mutex_t status_init_mutex_;
    void status_init_mutex_enter(void);
    void status_init_mutex_exit(void);*/


    // [service attributes] ==> Cscene_sim
    ros::ServiceServer reset_server_;
    bool resetCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    pthread_mutex_t reset_mutex_;
    void reset_mutex_enter(void);
    void reset_mutex_exit(void);

    // server to initialice person positions.
    ros::ServiceServer init_simulations_server_;
    bool init_simulationsCallback(iri_perception_msgs::InitialiceSim::Request &req, iri_perception_msgs::InitialiceSim::Response &res);
    pthread_mutex_t init_simulations_mutex_;
    void init_simulations_mutex_enter(void);
    void init_simulations_mutex_exit(void);



    // [client attributes]

    // [action server attributes]

    // [action client attributes]
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;
    move_base_msgs::MoveBaseGoal move_base_goal_;
    bool move_baseMakeActionRequest();
    void move_baseDone(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResultConstPtr& result);
    void move_baseActive();
    void move_baseFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
    bool isMoveBaseActive;
 		bool good_goal_;

    // [library attributes]
    Cplan_local_nav planner_;
    Cplan_local_nav::plan_mode plan_mode_;
    Spose  robot_pose_;
    Spose  robot_pose_tibi_;
    Spose  robot_pose_other_comp_pers_;
    Spose  robot_pose_2personGroup_; // NO SE USA! inicializada aparte. Es solo persona simulada sin odometria.
    bool group_simulation_now_; // true to simulate the group of two people and 1 robot. Else only one person comopanion.
    Spoint robot_goal_;
    Spose best_next_pose_companion_markers_;
    std::vector<Spoint> laser_points;
    std::vector<SdetectionObservation> obs;
    std::string force_map_path_, destination_map_path_;
    std::string robot_;
    std::string fixed_frame;
    std::string robot_frame;
    costmap_2d::Costmap2DROS* costmap_ros_;
    tf::TransformListener* tf_;
    bool initialized_;
    bool initialized2_;
    bool setup_;
    void init();
    std::vector<geometry_msgs::PoseStamped> global_plan_,sliced_global_plan_;//global coordinates
    std::vector<geometry_msgs::PoseStamped> local_plan_;//local coordinates
    double xy_goal_tolerance, v_goal_tolerance;

     bool transformGlobalPlan(const tf::TransformListener& tf, 
                              const std::vector<geometry_msgs::PoseStamped>& global_plan, 
                              const costmap_2d::Costmap2DROS& costmap, 
                              const std::string& global_frame, 
                              std::vector<geometry_msgs::PoseStamped>& transformed_plan);

     bool transformPose(const tf::TransformListener& tf, 
                                      const geometry_msgs::PoseStamped& plan_pose, 
                                      const costmap_2d::Costmap2DROS& costmap, 
                                      const std::string& target_frame, 
                                      geometry_msgs::PoseStamped& transformed_pose);

    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub);

    std::vector<Spoint> scan2points(const sensor_msgs::LaserScan scan);

    dynamic_reconfigure::Server<iri_simulated_person_companion_akp_local_planner::AkpLocalPlannerConfig> *dsrv_;
    iri_simulated_person_companion_akp_local_planner::AkpLocalPlannerConfig default_config_;

  /*  void init_force_planner_and_markers();
    void fill_scene_markers();
    void fill_best_path_2d();
    void fill_forces_markers();
    void fill_laser_obstacles();
    void fill_planning_markers_2d();
    void fill_planning_markers_3d();
    void fill_planning_markers_3d_companion();
    void fill_people_prediction_markers_2d();
    void fill_people_prediction_markers_2d_companion();
    void fill_people_prediction_markers_3d();
    void fill_people_prediction_markers_3d_companion();
    void fill_my_covariance_marker( visualization_msgs::Marker& marker, const SpointV_cov& point );
    void fill_robot_companion_markers();
    void fill_test_markers();*/

    unsigned int vis_mode_;
    bool frozen_mode_, move_base;
    unsigned int text_markers_old_size_;
    
    Goal_providing_mode goal_providing_mode_;
    void slice_plan();
    
    double slicing_path_diff_orientation_;
    
    std::deque<double> velocities_;

    // companion variables (ely)
    geometry_msgs::PointStamped header_point_;
    std::string target_frame_id;
    std::string source_frame_id;
    tf::TransformListener tf_listener_;
    unsigned int id_person_companion_;
  
    //send a goal variables (ely) // [action client attributes]
   // move_base_msgs::MoveBaseGoal move_base_goal_;
   // actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;
    Spoint actual_goal;
    //bool isMoveBaseActive;
    // change initial position, variables.
    double robot_ini_pose_x_;
    double robot_ini_pose_y_;
    double robot_ini_pose_theta_;

  Cperson_abstract::companion_reactive reactive_;
    // ely visualization bool variables (see forces)
    bool robot_goal_force_marker, robot_person_forces_marker, robot_obstacles_forces_marker, robot_resultant_force_marker,      robot_companion_force_marker;
    
    bool debug_antes_subgoals_entre_AKP_goals_;
    tf::TransformListener* tf_listener2_;

    // ely functions to send a goal
    bool doGoal();
    void do_Cscene_sim();
   // bool move_baseMakeActionRequest();
    //void move_baseFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
   // void move_baseActive();
   // void move_baseDone(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResultConstPtr& result);
    
    bool external_goal_; // boolean to do a external goal from rviz or not.
    //bool debug_real_test_companion_; 
    bool debug_real_test_companion_robot_; 
    bool we_have_companion_person_;
    int iter;
/***  !!!CLAS planning Atributes!!!  (fin) ***/

/***  !!!CLAS Cscene_sim Atributes!!!  (ini) ***/



  enum simulation_mode {Normal=0, Density_incremental };
  int n_persons_;
  simulation_mode simulation_mode_;
	Cscene_sim scene_;
	void init_sim();
  void init_person_companion_markers();
  bool ini_companion_; // bool to ini person in companion to make groups of person.
 // bool debug_antes_subgoals_entre_AKP_goals_;
  bool debug_real_test_companion_;
  bool debug_person_companion_;
  double ini_person_theta;
  bool remove_targets_, freeze_;
  double person_companion_slicing_path_diff_orientation_;

  Cperson_bhmip  new_person_companion_position_;
  Cperson_bhmip  new_person_companion_position_2groupPers_;
  void fill_forces_markers();
  void fill_scene_markers();
  void fill_planning_markers_2d();
  void fill_planning_markers_3d();
  void fill_people_prediction_markers_2d();
  void fill_people_prediction_markers_3d();
  void fill_laser_obstacles();
  void fill_best_path_2d();
  void fill_my_covariance_marker( visualization_msgs::Marker& marker, const SpointV_cov& point );
  bool do_Cscene_sim(Spose& pose_person_companion);

// visualization markers for person_companion planning akp
    ros::Publisher personCompanionAkpMarkers_publisher_;
    visualization_msgs::MarkerArray personCompanion_MarkerArray_msg_;
    visualization_msgs::Marker personCompanion_pred_traj_marker_, 
      personCompanion_pred_traj2d_marker_,
      personCompanion_cylinder_marker_,
      personCompanion_planning_marker_,
      personCompanion_robot_goal_marker_,
      personCompanion_robot_subgoal_marker_,
      personCompanion_robot_marker_,
      personCompanion_workspace_marker_,
      personCompanion_planning_goals_marker_,
      personCompanion_best_path_marker_,
      personCompanion_best_path2d_marker_,
      personCompanion_laser_obstacle_marker_,
      personCompanion_force_marker_,
      personCompanion_force_goal_marker_,
      personCompanion_force_int_person_marker_,
      personCompanion_force_obstacle_marker_,
      personCompanion_force_int_robot_marker_,
      personCompanion_text_marker_,
      personCompanion_nd_path_marker_,
      personCompanion_nd_path2d_marker_,
      personCompanion_robot_goal_marker2_,
      personCompanion_robot_goal_marker3_,
			next_goal_marker_;

    ros::Publisher tracksMarkers_publisher_;
    visualization_msgs::MarkerArray MarkerArray_msg_;
	  visualization_msgs::Marker track_marker_, id_marker_;

    ros::Publisher tracks_publisher_;
    iri_perception_msgs::detectionArray peopleTrackingArray_msg_;

    bool person_companion_goal_force_marker, person_companion_person_forces_marker, person_companion_obstacles_forces_marker,   person_companion_resultant_force_marker, person_companion_companion_force_marker;


    ros::Publisher destinations_of_tracks_publisher_;
    iri_perception_msgs::detectionArray destinationsOfTracksArray_msg_;
    

  bool debug_forces_akp_person_companion_;
/***  !!!CLAS Cscene_sim Atributes!!!  (fin) ***/


    // id person goal to go the group.
      unsigned int id_person_goal_;
      bool debug_goal_person_;
      Cplan_local_nav::action_mode Action_ROS_;
      Cplan_local_nav::simulation_case Actual_case_ROS_;

      double distance_to_aproach_person_goal_;


      // variables for group of two people and one robot:
     std::string robot_companion_person_name_; // variable 2 robot=companion person simulation.
     std::string robot_companion_person_name_out_node_; // variable 2 robot=companion person simulation.
     bool we_habe_tracks_of_other_person_companion_;
     std::vector<SdetectionObservation> obs_other_person_companion_;

     bool we_habe_destinations_of_other_person_companion_;
     std::vector<Sdestination> destinations_other_person_companion_;

  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    AkpLocalPlanner(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~AkpLocalPlanner(void);
  /**
    * @brief  Constructs the ros wrapper
    * @param name The name to give this instance of the trajectory planner
    * @param tf A pointer to a transform listener
    * @param costmap The cost map to use for assigning costs to trajectories
    */
    AkpLocalPlanner(std::string name, tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros);

    /**
      * @brief  Constructs the ros wrapper
      * @param name The name to give this instance of the trajectory planner
      * @param tf A pointer to a transform listener
      * @param costmap The cost map to use for assigning costs to trajectories
      */
    void initialize(std::string name, tf::TransformListener* tf,
        costmap_2d::Costmap2DROS* costmap_ros);

    /**
      * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
      * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
      * @return True if a valid trajectory was found, false otherwise
      */
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    /**
      * @brief  Set the plan that the controller is following
      * @param orig_global_plan The plan to pass to the controller
      * @return True if the plan was updated successfully, false otherwise
      */
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    /**
      * @brief  Check if the goal pose has been achieved
      * @return True if achieved, false otherwise
      */
    bool isGoalReached();

  // variables to stop the node to evaluate the costs:
    states current_state; // variable indicating the current state of the node.

    ros::Subscriber cmd_vel_stop_subscriber_;
    void cmd_vel_stop_callback(const geometry_msgs::Twist::ConstPtr& msg);
    pthread_mutex_t cmd_vel_stop_mutex_;
    void cmd_vel_stop_mutex_enter(void);
    void cmd_vel_stop_mutex_exit(void);
    double debug_stop_node_to_evaluate_costs_;


  protected:
   /**
    * \brief template algorithm class
    *
    * This template class refers to an implementation of an specific algorithm
    * interface. Will be used in the derivate class to define the common 
    * behaviour for all the different implementations from the same algorithm.
    */
    AkpLocalPlannerAlgorithm alg_;

   /**
    * \brief public node handle communication object
    *
    * This node handle is going to be used to create topics and services within
    * the node namespace. Additional node handles can be instantatied if 
    * additional namespaces are needed.
    */
    ros::NodeHandle public_node_handle_;

   /**
    * \brief private node handle object
    *
    * This private node handle will be used to define algorithm parameters into
    * the ROS parametre server. For communication pruposes please use the 
    * previously defined node_handle_ object.
    */
    ros::NodeHandle private_node_handle_;

   /**
    * \brief dynamic reconfigure server callback
    * 
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must 
    * implement it.
    *
    * \param config an object with new configuration from all algorithm 
    *               parameters defined in the config file.
    * \param level  integer referring the level in which the configuration
    *               has been changed.
    */
    void reconfigureCallback(iri_simulated_person_companion_akp_local_planner::AkpLocalPlannerConfig &config, uint32_t level);

    // [diagnostic functions]
    
    // [test functions]
};

#endif
