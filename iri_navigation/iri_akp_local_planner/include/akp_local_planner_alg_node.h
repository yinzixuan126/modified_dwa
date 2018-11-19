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
#include <iri_akp_local_planner/AkpLocalPlannerConfig.h>

// [publisher subscriber headers]
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <iri_perception_msgs/detectionArray.h>
#include <visualization_msgs/MarkerArray.h>

// [service client headers]

// [action server client headers]

// [libraries]
#include "nav/plan_local_nav.h"

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

    ros::Publisher g_plan_pub_;
    ros::Publisher l_plan_pub_;

    pthread_mutex_t planner_mutex_;
    void planner_mutex_enter(void);
    void planner_mutex_exit(void);

    ros::Publisher markers_publisher_;
    visualization_msgs::MarkerArray MarkerArray_msg_;
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
      nd_path2d_marker_;

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

    // [action server attributes]

    // [action client attributes]

    // [library attributes]
    Cplan_local_nav planner_;
    Cplan_local_nav::plan_mode plan_mode_;
    Spose  robot_pose_;
    Spoint robot_goal_;
    std::vector<Spoint> laser_points;
    std::vector<SdetectionObservation> obs;
    std::string force_map_path_, destination_map_path_;
    std::string robot_;
    std::string fixed_frame;
    std::string robot_frame;
    costmap_2d::Costmap2DROS* costmap_ros_;
    tf::TransformListener* tf_;
    bool initialized_;
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

    dynamic_reconfigure::Server<iri_akp_local_planner::AkpLocalPlannerConfig> *dsrv_;
    iri_akp_local_planner::AkpLocalPlannerConfig default_config_;

    void init_force_planner_and_markers();
    void fill_scene_markers();
    void fill_best_path_2d();
    void fill_forces_markers();
    void fill_laser_obstacles();
    void fill_planning_markers_2d();
    void fill_planning_markers_3d();
    void fill_people_prediction_markers_2d();
    void fill_people_prediction_markers_3d();
    void fill_my_covariance_marker( visualization_msgs::Marker& marker, const SpointV_cov& point );
    unsigned int vis_mode_;
    bool frozen_mode_, move_base;
    unsigned int text_markers_old_size_;
    
    Goal_providing_mode goal_providing_mode_;
    void slice_plan();
    
    double slicing_path_diff_orientation_;
    
    std::deque<double> velocities_;

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
    void reconfigureCallback(iri_akp_local_planner::AkpLocalPlannerConfig &config, uint32_t level);

    // [diagnostic functions]
    
    // [test functions]
};

#endif
