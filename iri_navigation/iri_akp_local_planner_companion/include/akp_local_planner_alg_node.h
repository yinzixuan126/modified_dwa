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

#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>

#include <dynamic_reconfigure/server.h>
#include <iri_akp_local_planner_companion/AkpLocalPlannerConfig.h>
 

// [publisher subscriber headers]
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <iri_perception_msgs/detectionArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h> // robot companion (ely) trasladar modelo tibi robot companion al marcker donde deberia estar el robot.

#include <sensor_msgs/Joy.h> // [Wii and PS3 comandaments]

#include <iri_perception_msgs/restartSim.h>

#include <random>
#include <iostream>
#include <fstream>      // std::ofstream
#include <string>
#include <stdio.h>
using namespace std;

// para el learning:
// [publisher subscriber headers]



////////////////


// [service client headers]
#include <iri_perception_msgs/InitialiceSim.h>
#include <move_base/move_base.h>

// [action server client headers]
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_broadcaster.h>

#include <wiimote/State.h>

////////////////

// [service client headers]



// [libraries]
#include "nav/plan_local_nav.h"


// [Wii and PS3 comandaments]
#include <iri_common_drivers_msgs/ps3state.h>
#define BUTTON_CANCEL_GOAL           iri_common_drivers_msgs::ps3state::PS3_BUTTON_SELECT
#define BUTTON_DEAD_MAN              iri_common_drivers_msgs::ps3state::PS3_BUTTON_REAR_RIGHT_1     //R1
//#define BUTTON_TRANS_SPEED_UP        iri_common_drivers_msgs::ps3state::PS3_BUTTON_ACTION_TRIANGLE
#define BUTTON_TRANS_SPEED_DOWN      iri_common_drivers_msgs::ps3state::PS3_BUTTON_ACTION_CROSS
#define BUTTON_ROT_SPEED_UP          iri_common_drivers_msgs::ps3state::PS3_BUTTON_ACTION_CIRCLE
#define BUTTON_ROT_SPEED_DOWN        iri_common_drivers_msgs::ps3state::PS3_BUTTON_ACTION_SQUARE
#define AXIS_TRANS_FORWARD           iri_common_drivers_msgs::ps3state::PS3_AXIS_STICK_LEFT_UPWARDS    //L3
#define AXIS_ROT_LEFTWARD            iri_common_drivers_msgs::ps3state::PS3_AXIS_STICK_RIGHT_LEFTWARDS //R3
#define BETA_UP                      iri_common_drivers_msgs::ps3state::PS3_BUTTON_CROSS_UP 
#define BETA_DOWN                    iri_common_drivers_msgs::ps3state::PS3_BUTTON_CROSS_DOWN 
#define NEAR_PERS_COMP               iri_common_drivers_msgs::ps3state::PS3_BUTTON_ACTION_TRIANGLE
#include <geometry_msgs/Twist.h>

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
    // [publisher attributes]
    ros::Publisher cost_params_publisher_;
    std_msgs::Float64MultiArray Float64_msg_;
    
    ros::Publisher cmd_vel_publisher_;
    geometry_msgs::Twist cmd_vel_msg_;

    // [publisher attributes]
    //ros::Publisher status_init_publisher_;
    //iri_perception_msgs::restartSim status_init_msg_;

    ros::Publisher g_plan_pub_;
    ros::Publisher l_plan_pub_;

    pthread_mutex_t planner_mutex_;
    void planner_mutex_enter(void);
    void planner_mutex_exit(void);

    ros::Publisher markers_publisher_;
    ros::Publisher markers_publisher_m2_;
    ros::Publisher markers_publisher_comp_markers_;
    ros::Publisher markers_publisher_people_prediction_time_;

    visualization_msgs::MarkerArray MarkerArray_msg_;
    visualization_msgs::MarkerArray MarkerArray_msg_comp_markers_;
    visualization_msgs::MarkerArray MarkerArray_msg_m2_;
    visualization_msgs::MarkerArray MarkerArray_msg_people_prediction_time_;

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
	    next_goal_marker_,
      robot_goal_to_position_respect_to_the_person,
      robot_goal_to_follow_the_path,
      final_robot_goal_act_iteration,
      pred_traj_point_marker_,
      medium_point_face_person_;

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


    // [service attributes]

    // [client attributes]
    ros::ServiceClient init_simulations_client_;
    ros::ServiceClient get_plan_client_;

    iri_perception_msgs::InitialiceSim init_simulations_srv_;

    nav_msgs::GetPlan get_plan_srv_;

    // [action server attributes]
    // [action client attributes]

    // [library attributes]
    Cplan_local_nav planner_;
    Cplan_local_nav::plan_mode plan_mode_;
    Spose  robot_pose_;
    Spose  robot_pose_from_odom_;
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

    dynamic_reconfigure::Server<iri_akp_local_planner_companion::AkpLocalPlannerConfig> *dsrv_;
    iri_akp_local_planner_companion::AkpLocalPlannerConfig default_config_;

    void init_force_planner_and_markers();
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
    void fill_my_covariance_marker( visualization_msgs::Marker& marker, const SpointV_cov& point , unsigned int track_id);
    void fill_robot_companion_markers();
    void fill_test_markers();
    void fill_people_prediction_markers_path_points();
    void fill_my_prediction_points( visualization_msgs::Marker& marker, const SpointV_cov& point );

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
  
double START_time_secs_fill_markers;
 	bool simulation_; // true if we have on simulation mode. False if we have in real robot mode.

    //send a goal variables (ely) // [action client attributes]
    move_base_msgs::MoveBaseGoal move_base_goal_;
    move_base_msgs::MoveBaseGoal before_good_move_base_goal_;
	//Spose before_move_base_goal_spoint_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;

	//actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_check_psible_path_;

	//actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> check_posible_path_move_base_;
//nav_core::BaseGlobalPlanner check_posible_path_move_base_;

    Spoint actual_goal;
    bool isMoveBaseActive;
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
	bool get_scout_results_doGoal_;
    bool doGoal(); // bool doGoal(Spoint goal);
    bool doGoal2(); // bool doGoal(Spoint goal);
    bool move_baseMakeActionRequest();
    void move_baseFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
    void move_baseActive();
    void move_baseDone(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResultConstPtr& result);
    

    bool external_goal_; // boolean to do a external goal from rviz or not.
    bool debug_real_test_companion_; 
    bool debug_real_test_companion_robot_; 
    bool we_have_companion_person_;
    int iter;

    // for wii and PS3 comandaments
    // [subscriber attributes]
    ros::Subscriber joy_subscriber_;
    void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg);
    pthread_mutex_t joy_mutex_;
    void joy_mutex_enter(void);
    void joy_mutex_exit(void);
    
    void useWii(std::vector<int> current_buttons, std::vector<int> last_buttons);
    void useWiiButton(const unsigned int & index);
    void usePs3(std::vector<int> current_buttons, std::vector<int> last_buttons, std::vector<float> current_axes);
    void usePs3Button(const unsigned int & index);
    void function_joy_teleop_mainNodeThread(void);

    bool human_is_alive_;

    double vt_max; // estas en teoria se eliminaran o las cambiare por las que a mi me hacen falta, pq son variables para teleop.
    double vr_max;
    double trans_speed_scale;
    double rot_speed_scale;
    bool cancel_goal;

    bool use_default_wii_button_;
    bool use_default_PS3_button_;


    bool check_execution_times_;
    int iterator_facke_;

	bool flag_play_change_id_;
   /**
    * \brief config variable
    *
    * This variable has all the parameters defined in the cfg config file.
    * Is updated everytime function node_config_update() is called.
    */
    //Config config_;
    iri_akp_local_planner_companion::AkpLocalPlannerConfig config_;
    /**
    * \brief Resets joy_watchdog time
    */
    void reset_joy_watchdog(void);
    /**
    * \brief Returns true if joy_watchdog timeouts
    */
    bool joy_watchdog_active(void);
    /**
    * \brief Updates joy_watchdog time
    */
    void update_joy_watchdog(void);
    /**
    * \brief Watchdog timeout duration
    */
    ros::Duration joy_watchdog_duration;
    /**
    * \brief Watchdog access mutex
    */
    CMutex joy_watchdog_access;

    // variables to stop the node to evaluate the costs:
    states current_state; // variable indicating the current state of the node.

    ros::Subscriber cmd_vel_stop_subscriber_;
    void cmd_vel_stop_callback(const geometry_msgs::Twist::ConstPtr& msg);
    pthread_mutex_t cmd_vel_stop_mutex_;
    void cmd_vel_stop_mutex_enter(void);
    void cmd_vel_stop_mutex_exit(void);
    double debug_stop_node_to_evaluate_costs_;

    bool fuera_bolitas_goals_companion_markers_;

    // variables for goal to person goal.
    // id person goal to go the group.
      unsigned int id_person_goal_;
      //unsigned int prueba_count;
    Cplan_local_nav::action_mode Action_ROS_;
    Cplan_local_nav::simulation_case Actual_case_ROS_;


// inicio variables para learning parameters:
unsigned int j_;
unsigned int i_;
std::vector<string> folder_names;
double folder_names_size;
bool out_comments;
bool output_view_get_from_file;
Spose robot_simulated_pose_;

double frecuencia_simulador_;
    geometry_msgs::PoseWithCovariance  pose;
    geometry_msgs::TwistWithCovariance twist;
    std::string tf_prefix_;
    std::string odom_id_;
    std::string base_link_id_;
    std::string base_footprint_id_;
    ros::Publisher odom_publisher_;
    nav_msgs::Odometry Odometry_msg_;
// FIN variables para learning parameters:


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
    void reconfigureCallback(iri_akp_local_planner_companion::AkpLocalPlannerConfig &config, uint32_t level);

    // [diagnostic functions]
    
    // [test functions]


// #define PS3_BUTTON_SELECT            0
// #define PS3_BUTTON_STICK_LEFT        1
// #define PS3_BUTTON_STICK_RIGHT       2
// #define PS3_BUTTON_START             3
// #define PS3_BUTTON_CROSS_UP          4
// #define PS3_BUTTON_CROSS_RIGHT       5
// #define PS3_BUTTON_CROSS_DOWN        6
// #define PS3_BUTTON_CROSS_LEFT        7
// #define PS3_BUTTON_REAR_LEFT_2       8
// #define PS3_BUTTON_REAR_RIGHT_2      9
// #define PS3_BUTTON_REAR_LEFT_1       10
// #define PS3_BUTTON_REAR_RIGHT_1      11
// #define PS3_BUTTON_ACTION_TRIANGLE   12
// #define PS3_BUTTON_ACTION_CIRCLE     13
// #define PS3_BUTTON_ACTION_CROSS      14
// #define PS3_BUTTON_ACTION_SQUARE     15
// #define PS3_BUTTON_PAIRING           16
// 
// #define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
// #define PS3_AXIS_STICK_LEFT_UPWARDS      1
// #define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
// #define PS3_AXIS_STICK_RIGHT_UPWARDS     3
// #define PS3_AXIS_BUTTON_CROSS_UP         4
// #define PS3_AXIS_BUTTON_CROSS_RIGHT      5
// #define PS3_AXIS_BUTTON_CROSS_DOWN       6
// #define PS3_AXIS_BUTTON_CROSS_LEFT       7
// #define PS3_AXIS_BUTTON_REAR_LEFT_2      8
// #define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
// #define PS3_AXIS_BUTTON_REAR_LEFT_1      10
// #define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
// #define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
// #define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
// #define PS3_AXIS_BUTTON_ACTION_CROSS     14
// #define PS3_AXIS_BUTTON_ACTION_SQUARE    15
// #define PS3_AXIS_ACCELEROMETER_LEFT      16
// #define PS3_AXIS_ACCELEROMETER_FORWARD   17
// #define PS3_AXIS_ACCELEROMETER_UP        18
// #define PS3_AXIS_GYRO_YAW                19
// 
// #define BUTTON_DEAD_MAN              PS3_BUTTON_REAR_RIGHT_1
// #define BUTTON_TRANS_SPEED_UP        PS3_BUTTON_ACTION_TRIANGLE
// #define BUTTON_TRANS_SPEED_DOWN      PS3_BUTTON_ACTION_CROSS
// #define BUTTON_ROT_SPEED_UP          PS3_BUTTON_ACTION_CIRCLE
// #define BUTTON_ROT_SPEED_DOWN        PS3_BUTTON_ACTION_SQUARE
// #define AXIS_TRANS_FORWARD           PS3_AXIS_STICK_LEFT_UPWARDS
// #define AXIS_ROT_LEFTWARD            PS3_AXIS_STICK_RIGHT_LEFTWARDS

// para los botones de la wii:
// http://docs.ros.org/jade/api/wiimote/html/msg/State.html
//int8    INVALID       = -1
//float32 INVALID_FLOAT = -1.0

//int8 MSG_BTN_1     = 0
//int8 MSG_BTN_2     = 1
//int8 MSG_BTN_A     = 2
//int8 MSG_BTN_B     = 3
//int8 MSG_BTN_PLUS  = 4
//int8 MSG_BTN_MINUS = 5
//int8 MSG_BTN_LEFT  = 6
//int8 MSG_BTN_RIGHT = 7
//int8 MSG_BTN_UP    = 8
//int8 MSG_BTN_DOWN  = 9
//int8 MSG_BTN_HOME  = 10
//int8 MSG_BTN_Z     = 0
//int8 MSG_BTN_C     = 1
//int8 MSG_CLASSIC_BTN_X       = 0
//int8 MSG_CLASSIC_BTN_Y       = 1
//int8 MSG_CLASSIC_BTN_A       = 2
//int8 MSG_CLASSIC_BTN_B       = 3
//int8 MSG_CLASSIC_BTN_PLUS    = 4
//int8 MSG_CLASSIC_BTN_MINUS   = 5
//int8 MSG_CLASSIC_BTN_LEFT    = 6
//int8 MSG_CLASSIC_BTN_RIGHT   = 7
//int8 MSG_CLASSIC_BTN_UP      = 8
//int8 MSG_CLASSIC_BTN_DOWN    = 9
//int8 MSG_CLASSIC_BTN_HOME    = 10
//int8 MSG_CLASSIC_BTN_L       = 11
//int8 MSG_CLASSIC_BTN_R       = 12
//int8 MSG_CLASSIC_BTN_ZL      = 13
//int8 MSG_CLASSIC_BTN_ZR      = 14

};

#endif
