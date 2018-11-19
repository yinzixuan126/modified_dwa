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

#ifndef _people_simulation_alg_node_h_
#define _people_simulation_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "people_simulation_alg.h"
#include "scene_sim.h"
// [libraries]
#include "nav/plan_local_nav.h"

#include <tf/transform_listener.h>

// [publisher subscriber headers]
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <iri_perception_msgs/detectionArray.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>

// [service client headers]
#include <std_srvs/Empty.h>

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class PeopleSimulationAlgNode : public algorithm_base::IriBaseAlgorithm<PeopleSimulationAlgorithm>
{
    enum simulation_mode {Normal=0, Density_incremental };
    enum Goal_providing_mode { Crop_local_window=0, Slicing_global}; // is for planning
  private:
    // [publisher attributes]
    ros::Publisher tracksMarkers_publisher_;
    visualization_msgs::MarkerArray MarkerArray_msg_;
	  visualization_msgs::Marker track_marker_, id_marker_;

    ros::Publisher tracks_publisher_;
    iri_perception_msgs::detectionArray peopleTrackingArray_msg_;

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
      personCompanion_robot_goal_marker3_;

      bool person_companion_goal_force_marker, person_companion_person_forces_marker, person_companion_obstacles_forces_marker, person_companion_resultant_force_marker, person_companion_companion_force_marker;

    // [subscriber attributes]
    ros::Subscriber fscan_subscriber_;// (ely) include obstacles in people simulation
    void fscan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    pthread_mutex_t fscan_mutex2_;
    void fscan_mutex_enter2(void);
    void fscan_mutex_exit2(void);
    sensor_msgs::LaserScan fscan;
    bool fscan_received;

    ros::Subscriber rscan_subscriber_; // (ely) include obstacles in people simulation
    void rscan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    pthread_mutex_t rscan_mutex2_;
    void rscan_mutex_enter2(void);
    void rscan_mutex_exit2(void);
    sensor_msgs::LaserScan rscan;
    bool rscan_received;


    // [service attributes]
    ros::ServiceServer reset_server_;
    bool resetCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    pthread_mutex_t reset_mutex_;
    void reset_mutex_enter(void);
    void reset_mutex_exit(void);

    // [client attributes]

    // [action server attributes]

    // [action client attributes]

	// [class atributes]
	int n_persons_;
  simulation_mode simulation_mode_;
	Cscene_sim scene_;
	void init_sim();
  void init_person_companion_markers();

  void fill_forces_markers();
  void fill_scene_markers();
  void fill_planning_markers_2d();
  void fill_planning_markers_3d();
  void fill_people_prediction_markers_2d();
  void fill_people_prediction_markers_3d();
  void fill_laser_obstacles();
  void fill_best_path_2d();
  void fill_my_covariance_marker( visualization_msgs::Marker& marker, const SpointV_cov& point );

	std::string force_map_path_, destination_map_path_;
	bool vis_mode_,remove_targets_, freeze_;
    tf::TransformListener tf_listener_;
    tf::TransformListener* tf_listener2_;
    std::string robot_;
  bool ini_companion_; // bool to ini person in companion to make groups of person.

  // (ely) variables to include fake laser (fake obstacles) in people simulation
  std::vector<Spoint> laser_points; 
  std::vector<Spoint> scan2points(const sensor_msgs::LaserScan scan); 
  std::string fixed_frame;
  Spose robot_pose_; // aqui es la person_companion_pose ... arreglar...

   //Cplan_local_nav planner_;
  bool debug_antes_subgoals_entre_AKP_goals_;
  bool debug_real_test_companion_;
  void slice_plan();
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
  Goal_providing_mode person_companion_goal_providing_mode_;
  bool initialized_;
  double person_companion_slicing_path_diff_orientation_;
  unsigned int text_markers_old_size_;
  std::vector<SdetectionObservation> obs;

  std::vector<geometry_msgs::PoseStamped> global_plan_,sliced_global_plan_;
  //geometry_msgs::PointStamped header_point_;

  bool debug_person_companion_;
  Cperson_bhmip  new_person_companion_position_;

   unsigned int vis_mode2_;
   double ini_person_theta;

 public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    PeopleSimulationAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~PeopleSimulationAlgNode(void);

  protected:
   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the algorithm is on running state. Loop frequency 
    * can be tuned by modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects 
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
    void mainNodeThread(void);

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
    void node_config_update(Config &config, uint32_t level);

   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the 
    * specific algorithms may be added.
    */
    void addNodeDiagnostics(void);

    // [diagnostic functions]
    
    // [test functions]
};

#endif
