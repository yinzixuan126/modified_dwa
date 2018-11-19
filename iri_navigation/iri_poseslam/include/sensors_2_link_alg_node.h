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

#ifndef _sensors_2_link_alg_node_h_
#define _sensors_2_link_alg_node_h_

#include <Eigen/Dense>

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "sensors_2_link_alg.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// [publisher subscriber headers]
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// [service client headers]
#include <iri_poseslam/GetLink.h>
#include <iri_laser_icp/GetRelativePose.h>

// [action server client headers]

using namespace Eigen;

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class Sensors2LinkAlgNode : public algorithm_base::IriBaseAlgorithm<Sensors2LinkAlgorithm>
{
  private:
    // [publisher attributes]
    
    // [subscriber attributes]
    ros::Subscriber scan_subscriber_;
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    
    ros::Subscriber odom_relative_subscriber_;
    void odom_relative_callback(const nav_msgs::Odometry::ConstPtr& msg);

    ros::Subscriber cmd_vel_subscriber_;
    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
    
    // [service attributes]
    ros::ServiceServer get_link_server_;
    bool get_linkCallback(iri_poseslam::GetLink::Request &req, iri_poseslam::GetLink::Response &res);

    // [client attributes]
    ros::ServiceClient get_relative_pose_client_;
    iri_laser_icp::GetRelativePose get_relative_pose_srv_;

    // [action server attributes]

    // [action client attributes]
    
    // VARIABLES
    // Last laser scans
    sensor_msgs::LaserScan last_laser_scan_, ready_laser_scan_;
    // State booleans
    bool new_laser_scan_, fusion_ready_, stopped_since_last_odom_, currently_stopped_;
    uint ready_odom_id_, prev_seq_, laser_scan_counter_;
    // Accumulated relative odometry
    Vector3d odom_rel_, d_local_;
    Matrix3d odom_rel_cov_, Jp_, Jd_, Q_;
    ros::Time odom_rel_time_;
    // Buffers
    std::vector<geometry_msgs::PoseWithCovarianceStamped> odom_buffer_;
    std::vector<sensor_msgs::LaserScan> laser_scan_buffer_;
    std::vector<Vector3d> odom_rel_buffer_;
    std::vector<Matrix3d> odom_rel_cov_buffer_;
    std::vector<ros::Time> odom_rel_time_buffer_;
    // TF
    tf::Transform T_base_laser_, T_laser_base_;
    std::string laser_frame_id_, base_frame_id_;
    bool tf_ready_;
    //Parameters
    bool online_mode_, allow_slipping_;
    double ICP_covariance_correction_factor_;
    uint fusion_mode_, N_scans_discard_;
    
  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    Sensors2LinkAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~Sensors2LinkAlgNode(void);

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
    * \brief offline odometry
    *
    * This function computes the odometry in offline mode.
    * \return the odometry link
    */
   iri_poseslam::GetLink::Response offline_odometry();

   /**
    * \brief online odometry
    *
    * This function computes the odometry in online mode.
    * \return the odometry link
    */
   iri_poseslam::GetLink::Response online_odometry();

   /**
    * \brief odometry fusion
    *
    * In this function the relative odometry and the ICP odometry are fused. 
    * An interpolation of the relative odometry is also done.
    * \return pose with covariance stamped with the odometry information
    */
   geometry_msgs::PoseWithCovarianceStamped odometry_fusion(const sensor_msgs::LaserScan &laser_scan, const int &odom_rel_idx);

    /**
    * \brief interpolate odometry
    *
    * This function computes the interpolation of two relative odometries in a midle stamp.
    * \param odom_rel_idx the index of the last odometry to interpolate in the buffer
    * \param scan_stamp the stamp of the moment of interpolation
    * \return odometry (pose and covariance) interpolated
    */
   void interpolate_odom_rel(const int &odom_rel_idx, const ros::Time scan_stamp, Vector3d& int_odom_rel, Matrix3d& int_odom_rel_cov);

   /**
    * \brief from base footprint frame to laser frame
    *
    * This function changes a vector from base frame to laser frame.
    * \param old_d original vector in base frame.
    * \return the vector in the laser frame.
    */
    geometry_msgs::Pose base_2_laser_frame(const geometry_msgs::Pose &base_disp);

    geometry_msgs::PoseWithCovariance laser_2_base_frame(const geometry_msgs::PoseWithCovariance &laser_disp_cov);
    void laser_2_base_frame(const geometry_msgs::PoseWithCovariance &laser_disp_cov, Vector3d &odom_ICP, Matrix3d &odom_ICP_cov);
    //Vector3d base_2_laser_frame(const Vector3d &old_d);

    //void laser_2_base_frame(Vector3d &odom_ICP, Matrix3d &odom_ICP_cov);

    //Vector3d laser_2_base_frame(const Vector3d &odom_ICP);

    //geometry_msgs::Pose base_2_laser_frame(const geometry_msgs::Pose &base_disp);
    //Vector3d base_2_laser_frame(const Vector3d &old_d);
    /**
    * \brief from laser frame to base footprint frame
    *
    * This function changes the ICP link to the base footprint frame.
    * \param odom_ICP the vector containing the link
    * \param odom_ICP_cov the covariance matrix of the link
    */
    //void laser_2_base_frame(const geometry_msgs::PoseWithCovariance &laser_disp_cov, Vector3d &odom_ICP, Matrix3d &odom_ICP_cov);
    //void laser_2_base_frame(Vector3d &odom_ICP, Matrix3d &odom_ICP_cov);
    //geometry_msgs::Pose laser_2_base_frame(const geometry_msgs::Pose &laser_disp);

    /**
    * \brief from laser frame to base footprint frame
    *
    * This function changes a vector from laser frame to the base footprint frame.
    * \param odom_ICP the vector in laser frame
    * \return the vector in the base frame.
    */
    //Vector3d laser_2_base_frame(const Vector3d &odom_ICP);

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
   
    // CONVERSION FUNCTIONS (EIGEN - ROS_MSGS)

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
    boost::array<double, 36> matrix_2_covariance(const Matrix3d &cov) const;
   /**
    * \brief matrix and vector to posewithcovariance
    *
    * This function converts a pose (eigen::Vector3d) and a covariance (eigen::Matrix3d)
    * to a posewithcovariance ros message.
    * \param p pose vector
    * \param cov covariance matrix
    * \return posewithcovariance message
    */
   geometry_msgs::Pose vector_2_pose(const Vector3d &p) const;

    void load_tf();

    bool is_semiPD(const Matrix3d& M) const;
    // [diagnostic functions]
    
    // [test functions]
};

#endif
