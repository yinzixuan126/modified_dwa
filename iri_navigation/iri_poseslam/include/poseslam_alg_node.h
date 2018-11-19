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

#ifndef _poseslam_alg_node_h_
#define _poseslam_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "poseslam_alg.h"

// [publisher subscriber headers]
#include <iri_poseslam/Trajectory.h>

// [service client headers]
#include <iri_poseslam/GetLink.h>

// [action server client headers]

//#include <std_msgs/Header.h>

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class PoseslamAlgNode : public algorithm_base::IriBaseAlgorithm<PoseslamAlgorithm>
{
  private:
    // [publisher attributes]
    ros::Publisher trajectory_publisher_;
    iri_poseslam::Trajectory Trajectory_msg_;
    std::vector<uint> loops_;
    std::vector<VectorXd> Trajectory_poses_;

    // [subscriber attributes]

    // [service attributes]

    // [client attributes]
    ros::ServiceClient get_link_client_;
    iri_poseslam::GetLink get_link_srv_;
    
    // [action server attributes]

    // [action client attributes]
    
    // POSE SLAM
    uint step_;
    uint ended;
    double dz_footprint_2_base_;
    std::string results_path_;
    
  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    PoseslamAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~PoseslamAlgNode(void);

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
    * \brief augment trajectories
    *
    * Update the trajectories messages adding the new pose.
    * 
    * \param header of the new pose 
    */
    void augment_trajectories(const std_msgs::Header& header);

    /**
    * \brief update trajectories
    *
    * Update the trajectories messages.
    * 
    * \param LoopClosed boolean true if have been a Loop Closure 
    */
    void update_trajectories(const bool& LoopClosed);

    /**
    * \brief print results
    *
    * Prints the time and covariances results in a matlab file.
    */
    void print_results() const;

    /**
    * \brief recompute trajectory
    *
    * Update the trajectory message after a loop closure
    * 
    */
    void recompute_trajectory();
    
    /**
    * \brief recompute segment
    *
    * recompute the segment of redundant poses betwen non-redundant poses after a loop closure
    * \param new initial non-redundant pose
    * \param new final non-redundant pose
    * \param index of initial pose
    * \param index of final pose
    */
    std::vector<VectorXd> recompute_segment(const VectorXd& new_initial_pose, const VectorXd& new_final_pose, const int& initial_step, const int& final_step);

    /**
    * \brief create posewithcovariacestamped
    *
    * create a posewithcovariancestamped from:
    * \param header
    * \param vector of the pose
    * \param std::vector<double> of the covariance
    */
    geometry_msgs::PoseWithCovarianceStamped create_PoseWithCovarianceStamped(const std_msgs::Header& header, const VectorXd& last_pose, const std::vector<double>& last_cov);

    /**
    * \brief create posewithcovariace
    *
    * create a posewithcovariance from:
    * \param vector of the pose
    * \param std::vector<double> of the covariance
    */
    geometry_msgs::PoseWithCovariance create_PoseWithCovariance(const VectorXd& last_pose, const std::vector<double>& last_cov);
    
    /**
     * \brief rotation matrix
     *
     * This function returns a 2D (x,y,theta) rotation matrix
     * of angle 'alpha'.
     * \param alpha angle of rotation in radiants
     * \param odom_rel_idx index of the relative odom for interpolation
     * \return the rotation matrix
     */
    MatrixXd rotation_matrix(const double &alpha) const;

    /**
     * \brief pi to pi
     *
     * This function returns the angle given in the (-pi, pi] interval
     * \param angle
     * \return the (-pi, pi] angle
     */
    double pi_2_pi(const double& angle) const;

    // [diagnostic functions]
    
    // [test functions]
};

#endif
