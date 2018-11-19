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

#ifndef _trajectory_3Dscans_2_pointcloud_alg_node_h_
#define _trajectory_3Dscans_2_pointcloud_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "trajectory_3Dscans_2_pointcloud_alg.h"
#include <Eigen/Dense>
// [publisher subscriber headers]
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <iri_poseslam/Trajectory.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <laser_geometry/laser_geometry.h>
#include <math.h>
#include <queue>

// [service client headers]

// [action server client headers]

using namespace Eigen;

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class Trajectory3DScans2PointcloudAlgNode : public algorithm_base::IriBaseAlgorithm<Trajectory3DScans2PointcloudAlgorithm>
{
  private:

    Matrix4f T_laser_frame_;
    bool emptyPointCloud_;

    // [publisher attributes]
    ros::Publisher slices3D_pointcloud_publisher_;
    sensor_msgs::PointCloud2 PointCloud_msg_;
    
    // [subscriber attributes]
    ros::Subscriber slices3D_subscriber_;
    void slices3D_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    ros::Subscriber trajectory_subscriber_;
    void trajectory_callback(const iri_poseslam::Trajectory::ConstPtr& msg);

    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]

    // Other variables
    uint last_step_;
    bool new_trajectory_, last_loops_;
    iri_poseslam::Trajectory last_trajectory_;

    //Buffers
    std::vector<sensor_msgs::PointCloud2> trajectory_slices_;
    std::vector<std::pair<uint, double> > interpolation_buffer_;

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
    Trajectory3DScans2PointcloudAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~Trajectory3DScans2PointcloudAlgNode(void);

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
    
    /**
    * \brief recompute PointCloud msg
    *
    * Recompute the PointCloud message after a new trajectory msg received.
    * \param last trajectory message recieved
    */
    void update_pointcloud(const iri_poseslam::Trajectory& trajectory);

    /**
    * \brief transform point cloud
    *
    * Transform the pointcloud in relative coordinates to the global frame
    * \param pcloud The pointclout to convert 
    * \return PointCloud2
    */
    //sensor_msgs::PointCloud2 transform_point_cloud(const sensor_msgs::PointCloud2& pcloud, const geometry_msgs::Pose& pose);
    void transform_point_cloud(sensor_msgs::PointCloud2& pcloud, const geometry_msgs::Pose& pose);
    
    /**
    * \brief add to point cloud
    *
    * Add all points of a point cloud in the point cloud message
    * \param newPointCloud The PointCloud2 to add at PointCloud2 message
    */
    void add_to_PointCloud_msg(const sensor_msgs::PointCloud2& newPointCloud);
    
    /**
    * \brief laser scan to point cloud
    *
    * Clear the point cloud message
    */
    void clear_PointCloud_msg();
    
    /**
    * \brief interpole slice pose
    *
    * This function computes the pose of a slice interpolating poses of a trajectory message.
    * \param id of the slice from which compute the pose
    * \return pose of the slice
    */
    geometry_msgs::Pose interpole_slice_pose(const uint& id) const;
    
    /**
    * \brief search interpolation
    *
    * This function finds the index of the first of two poses of trajectory between the slice have been taken and the factor of interpolation
    * \param slice from which compute the interpolation
    * \return pair of uint index of the first pose and double the interpolation factor
    */
    std::pair<uint,double> search_interpolation(const sensor_msgs::PointCloud2& slice, const uint& prev_i) const;
    
    /**
    * \brief transformation matrix
    *
    * Create a 4x4 eigen matrix of the transformation from the local frame to global
    * \param floats x, y and z of the laser in global coordinates
    * \param float alpha orientation in global coordinates
    * \return Matrix4f
    */
    Matrix4f transformation_matrix(const float x, const float y, const float z, const float alpha) const;

    /**
     * \brief pi to pi
     *
     * This function returns the angle given in the (-pi, pi] interval
     * \param angle
     * \return the (-pi, pi] angle
     */
    double pi_2_pi(const double& angle) const;
};

#endif
