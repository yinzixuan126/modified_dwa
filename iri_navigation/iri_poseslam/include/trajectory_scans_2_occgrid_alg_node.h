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

#ifndef _trajectory_scans_2_occgrid_alg_node_h_
#define _trajectory_scans_2_occgrid_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "trajectory_scans_2_occgrid_alg.h"
#include <Eigen/Dense>
// [publisher subscriber headers]
#include <nav_msgs/OccupancyGrid.h>
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
class TrajectoryScans2OccGridAlgNode : public algorithm_base::IriBaseAlgorithm<TrajectoryScans2OccGridAlgorithm>
{
  private:

    // [publisher attributes]
    ros::Publisher occgrid_publisher_;
    bool publish_redundant_;

    // [subscriber attributes]
    ros::Subscriber scan_subscriber_;
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    ros::Subscriber trajectory_subscriber_;
    void trajectory_callback(const iri_poseslam::Trajectory::ConstPtr& msg);

    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]
    
    // Occupancy Grid Map
    nav_msgs::OccupancyGrid occupancy_grid_;
    Array<double, Dynamic, Dynamic, ColMajor> logodds_grid_;
    double Lfree_, Lobst_, Lobst_thres_, Lfree_thres_;
    int max_n_cells_;
    
    // Other variables
    uint last_step_;
    Vector2i n_cells_;
    Vector2f map_origin_;
    double grid_size_, laser_ray_incr_;
    bool new_trajectory_, last_loops_;
    iri_poseslam::Trajectory last_trajectory_;
    
    //Buffers
    std::queue<sensor_msgs::LaserScan> laser_scan_buffer_;
    std::vector<sensor_msgs::LaserScan> trajectory_scans_;
    // TF
    tf::Transform T_base_laser_, T_laser_base_;
    std::string laser_frame_id_, base_frame_id_;
    bool tf_ready_;

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
    TrajectoryScans2OccGridAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~TrajectoryScans2OccGridAlgNode(void);

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
    * \brief recompute trajectory scans
    *
    * Recompute the trajectory scans buffer after a new trajectory msg received.
    * \param last trajectory message recieved
    */
    void update_trajectory_scans(const iri_poseslam::Trajectory& trajectory);
    
    /**
    * \brief recompute occupancy grid
    *
    * Recompute the occupancy grid  after a new trajectory msg received.
    * \param last trajectory message recieved
    */
    void recompute_occupancy_grid(const iri_poseslam::Trajectory& trajectory);

    /**
    * \brief add laser scan to logodds
    *
    * Update with a laser scan message the logodds update in the occupancy log odds array
    * \param LScan The laser scan message to convert 
    * \param pose from where the laser scan was taken 
    */
    void add_scan_to_logodds(const sensor_msgs::LaserScan& LScan, const geometry_msgs::Pose& pose);

    /**
    * \brief add ray to logodds
    *
    * Adds the log odds of a laser scan ray message
    * \param theta angle of the ray
    * \param range of the ray
    * \param pose from where the laser scan was taken 
    */
    void add_ray_2_logodds(const double& theta, const double& range, const geometry_msgs::Pose& pose);
    
    /**
    * \brief vector 2 cell
    *
    * Returns the occupancy grid cell index of the position in the given by the vector
    * \param p The position vector
    * \return Vector containing the row and column
    */
    Vector2i vector2cell(const Vector2f& p);

     /**
    * \brief update occupancy grid
    *
    * Updates the occupancy grid msg.
    */
    void update_occupancy_grid();
    
    /**
    * \brief resize occupancy grid
    *
    * Updates the occupancy grid msg info when a resize have been done.
    */
    void resize_OccupancyGrid();
    
    /**
    * \brief resize map
    *
    * Resize in one dimension the occupancy and log_odds map
    * \param dim The dimension
    * \param oversize
    * \param place In front or back of the array
    */
    void resize_map(const int& dim, const uint& oversize, const bool& back);

    /**
    * \brief transformation matrix
    *
    * Create a 4x4 eigen matrix of the transformation from the local frame to global
    * \param floats x, y and z of the laser in global coordinates
    * \param float alpha orientation in global coordinates
    * \return Matrix4f
    */
    Matrix4f transformation_matrix(const float x, const float y, const float z, const float alpha) const;

    void load_tf();
};

#endif
