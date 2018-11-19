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

#ifndef _odom_2_odomrel_alg_node_h_
#define _odom_2_odomrel_alg_node_h_

#include <Eigen/Dense>
#include <iri_base_algorithm/iri_base_algorithm.h>
#include "odom_2_odomrel_alg.h"

#include <tf/transform_broadcaster.h>

// [publisher subscriber headers]
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// [service client headers]

// [action server client headers]

using namespace Eigen;

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class Odom2OdomrelAlgNode : public algorithm_base::IriBaseAlgorithm<Odom2OdomrelAlgorithm>
{
  private:
    // [publisher attributes]
    ros::Publisher odom_rel_publisher_;
    nav_msgs::Odometry Odometry_rel_msg_;

    // [subscriber attributes]
    ros::Subscriber odom_subscriber_;
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    
    // [service attributes]
    
    // [client attributes]
    
    // [action server attributes]

    // [action client attributes]
    
    Vector3d last_pose_;
    double covariance_factor_;
    nav_msgs::Odometry last_odom_;
    
  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    Odom2OdomrelAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~Odom2OdomrelAlgNode(void);

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

    /**
     * \brief rotation matrix
     *
     * This function returns a 2D (x,y,theta) rotation matrix
     * of angle 'alpha'.
     * \param alpha angle of rotation in radiants
     * \param odom_rel_idx index of the relative odom for interpolation
     * \return the rotation matrix
     */
    Matrix3d rotation_matrix(const double &alpha) const;
    
    /**
     * \brief covariance to matrix
     *
     * This function converts the covariance of a PoseWithCovariance ros message to a eigen::Matrix3d
     * \param pose PoseWithCovariance ros message
     * \return the covariance matrix
     */
    Matrix3d covariance_2_matrix(const geometry_msgs::PoseWithCovariance &pose) const;
    
    /**
     * \brief pose to vector
     *
     * This function converts the pose ros message to a eigen::Vector3d
     * \param pose pose ros message
     * \return the pose vector
     */
    Vector3d pose_2_vector(const geometry_msgs::Pose &pose) const;
    
    /**
     * \brief matrix and vector to posewithcovariance
     *
     * This function converts a pose (eigen::Vector3d) and a covariance (eigen::Matrix3d)
     * to a posewithcovariance ros message.
     * \param p pose vector
     * \param cov covariance matrix
     * \return posewithcovariance message
     */
    geometry_msgs::PoseWithCovariance eigen_2_posewithcovariance(const Vector3d &p, const Matrix3d &cov) const;

    /**
     * \brief change to base footprint frame
     *
     * This function changes the ICP link to the base footprint frame.
     * \param odom_ICP the vector containing the link
     * \param odom_ICP_cov the covariance matrix of the link
     */
    void change_2_base_footprint_frame(Vector3d &odom_ICP, Matrix3d &odom_ICP_cov);
    
    // [diagnostic functions]
    
    // [test functions]
};

#endif
