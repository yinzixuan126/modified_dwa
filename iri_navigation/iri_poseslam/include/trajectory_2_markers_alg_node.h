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

#ifndef _trajectory_2_markers_alg_node_h_
#define _trajectory_2_markers_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "trajectory_2_markers_alg.h"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

// [publisher subscriber headers]
#include <iri_poseslam/Trajectory.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class Trajectory2MarkersAlgNode : public algorithm_base::IriBaseAlgorithm<Trajectory2MarkersAlgorithm>
{
  private:
    // [publisher attributes]
    ros::Publisher CovarianceMarkers_publisher_;
    ros::Publisher TrajectoryMarkers_publisher_;
    ros::Publisher CurrentPoseMarker_publisher_;
    
    visualization_msgs::MarkerArray covariance_markers_;
    visualization_msgs::Marker loops_marker_;
    visualization_msgs::Marker trajectory_marker_;
    visualization_msgs::Marker current_marker_;

    std_msgs::ColorRGBA loop_color_, covariance_color_;
    
    // [subscriber attributes]
    ros::Subscriber trajectory_subscriber_;
    void trajectory_callback(const iri_poseslam::Trajectory::ConstPtr& msg);

    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]

    std::vector<bool> loop_step_;
    uint nLoops_;
    iri_poseslam::Trajectory last_trajectory_;
    bool new_trajectory_;

    // Mutex
    pthread_mutex_t last_trajectory_mutex_;
    void last_trajectory_mutex_enter(void);
    void last_trajectory_mutex_exit(void);

  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    Trajectory2MarkersAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~Trajectory2MarkersAlgNode(void);

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

    /** \brief fill marker array message  
    */
    void update_markers(const iri_poseslam::Trajectory& trajectory);

    /** \brief get trajectory marker
    */
    visualization_msgs::MarkerArray get_trajectory_marker() const;
    
    /** \brief create marker 
    */
    visualization_msgs::Marker create_marker(const uint& id, const std_msgs::Header& header, const Eigen::Matrix2d& covs, const double& theta_cov, const geometry_msgs::Point& position, const bool& loopClosure) const;
    
    /** \brief change current marker 
    */
    void change_current_marker(const Eigen::Matrix2d& covs, const double& theta_cov, const geometry_msgs::Point& position);
    
    /** \brief get ith xy cov 
    */
    Eigen::Matrix2d get_ith_cov(const iri_poseslam::Trajectory& msg, const uint i) const;
    
    /** \brief get ith theta cov 
    */
    double get_ith_theta_cov(const iri_poseslam::Trajectory& msg, const uint i) const;

    /** \brief get xy cov 
    */
    Eigen::Matrix2d get_cov(const geometry_msgs::PoseWithCovarianceStamped& p) const;

    /** \brief get theta cov 
    */
    double get_theta_cov(const geometry_msgs::PoseWithCovarianceStamped& p) const;
};

#endif
