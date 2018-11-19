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

#ifndef _wolf_alg_node_h_
#define _wolf_alg_node_h_

//iri base 
#include <iri_base_algorithm/iri_base_algorithm.h>

//this package
#include "wolf_alg.h"

// [publisher subscriber headers]
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class WolfAlgNode : public algorithm_base::IriBaseAlgorithm<WolfAlgorithm>
{
  private:
    bool draw_lines_;
    int window_length_, n_lasers_;
    double new_frame_elapsed_time_;
    
    //ceres
    ceres::Solver::Options ceres_options_;
    ceres::Problem::Options problem_options_;
    CeresManager* ceres_manager_;
    
    //Wolf: odom sensor
    SensorOdom2D* odom_sensor_ptr_;
    
    //Wolf: laser sensors
    std::vector<SensorLaser2D*> laser_sensor_ptr_;
    std::vector<bool> laser_params_set_;
    std::vector<bool> laser_tf_loaded_;
    std::vector<std::string> laser_frame_name_;
    std::map<std::string,unsigned int> laser_frame_2_idx_;

    //Wolf: laser processor algorithm
    laserscanutils::ExtractCornerParams corners_alg_params_;
    bool new_corners_alg_params_;

    //Wolf: manager
    WolfManager* wolf_manager_;
    
    //visualization
    std::vector<std_msgs::ColorRGBA> line_colors_;

    //transforms
    tf::TransformBroadcaster tfb_;
    tf::TransformListener    tfl_;
    tf::Transform T_map2base_; //wolf output
    tf::Transform T_odom2base_; //published by odom source
    tf::Transform T_map2odom_; //to be broadcasted by this node

    // [publisher attributes]
    ros::Publisher lines_publisher_;
    visualization_msgs::MarkerArray lines_MarkerArray_msg_;

    ros::Publisher constraints_publisher_;
    visualization_msgs::Marker constraints_Marker_msg_;

    ros::Publisher corners_publisher_;
    visualization_msgs::MarkerArray corners_MarkerArray_msg_;

    ros::Publisher vehicle_publisher_;
    visualization_msgs::MarkerArray vehicle_MarkerArray_msg_;

    // [subscriber attributes]

    //Odometry callback
    ros::Time last_odom_stamp_;
    ros::Subscriber odometry_subscriber_;
    void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);
    pthread_mutex_t odometry_mutex_;
    void odometry_mutex_enter(void);
    void odometry_mutex_exit(void);

    //Lidar callbacks
    std::vector<ros::Subscriber> laser_subscribers_;
    pthread_mutex_t laser_mutex_;
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void laser_mutex_enter();
    void laser_mutex_exit();
    std::string base_frame_name_;

    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]

  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    WolfAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~WolfAlgNode(void);

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

    void computeLaserScan(CaptureLaser2D* new_capture, const std_msgs::Header & header, const unsigned int laser_idx);

    void updateLaserParams(const unsigned int laser_idx, const sensor_msgs::LaserScan::ConstPtr& msg);

    void loadLaserTf(const unsigned int laser_idx);
};

#endif
