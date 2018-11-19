/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef ACKERMANN_PLANNER_H_
#define ACKERMANN__PLANNER_H_

#include <vector>
#include <Eigen/Core>

#include <iri_ackermann_local_planner/AckermannLocalPlannerConfig.h>
#include <ackermann_planner_util.h>

//for creating a local cost grid
#include <base_local_planner/map_grid_visualizer.h>
#include <pcl_ros/publisher.h>

//for obstacle data access
#include <costmap_2d/costmap_2d.h>

#include <base_local_planner/trajectory.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>
#include <ackermann_trajectory_generator.h>
#include <ackermann_odom_helper.h>

#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <heading_cost_function.h>
#include <ackermann_trajectory_search.h>

#include <nav_msgs/Path.h>

/**
 * @class AckermannPlanner
 * @brief A class implementing a local planner using the Dynamic Window Approach
 */
class AckermannPlanner 
{
  public:
    /**
     * @brief  Constructor for the planner
     * @param name The name of the planner 
     * @param costmap_ros A pointer to the costmap instance the planner should use
     * @param global_frame the frame id of the tf frame to use
     */
    AckermannPlanner(std::string name, AckermannPlannerUtil *planner_util);

    /**
     * @brief  Destructor for the planner
     */
    ~AckermannPlanner();

    /**
     * @brief Reconfigures the trajectory planner
     */
    void reconfigure(iri_ackermann_local_planner::AckermannLocalPlannerConfig &cfg);

    /**
     * @brief  Check if a trajectory is legal for a position/velocity pair
     * @param pos The robot's position
     * @param vel The robot's velocity
     * @param vel_samples The desired velocity
     * @return True if the trajectory is valid, false otherwise
     */
    bool check_trajectory(const Eigen::Vector3f pos,const Eigen::Vector3f ackermann,const Eigen::Vector2f samples);

    /**
     * @brief Given the current position and velocity of the robot, find the best trajectory to exectue
     * @param global_pose The current position of the robot 
     * @param global_vel The current velocity of the robot 
     * @param drive_velocities The velocities to send to the robot base
     * @return The highest scoring trajectory. A cost >= 0 means the trajectory is legal to execute.
     */
    base_local_planner::Trajectory find_best_path(tf::Stamped<tf::Pose> global_pose,TAckermannState ackermann,tf::Stamped<tf::Pose>& drive_velocities,std::vector<geometry_msgs::Point> footprint_spec);

    /**
     * @brief  Take in a new global plan for the local planner to follow, and adjust local costmaps
     * @param  new_plan The new global plan
     */
    void update_plan_and_local_costs(tf::Stamped<tf::Pose> global_pose,const std::vector<geometry_msgs::PoseStamped>& new_plan);

    /**
     * @brief Get the period at which the local planner is expected to run
     * @return The simulation period
     */
    double get_sim_period(void);

    /**
     * @brief Compute the components and total cost for a map grid cell
     * @param cx The x coordinate of the cell in the map grid
     * @param cy The y coordinate of the cell in the map grid
     * @param path_cost Will be set to the path distance component of the cost function
     * @param goal_cost Will be set to the goal distance component of the cost function
     * @param occ_cost Will be set to the costmap value of the cell
     * @param total_cost Will be set to the value of the overall cost function, taking into account the scaling parameters
     * @return True if the cell is traversible and therefore a legal location for the robot to move to
     */
    bool get_cell_costs(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost);

    /**
     * sets new plan and resets state
     */
    bool set_plan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  private:

    AckermannPlannerUtil *planner_util_;

    double stop_time_buffer_; ///< @brief How long before hitting something we're going to enforce that the robot stop
    double pdist_scale_, gdist_scale_, occdist_scale_,hdiff_scale_;
    Eigen::Vector2f vsamples_;

    double sim_period_;///< @brief The number of seconds to use to compute max/min vels for dwa
    base_local_planner::Trajectory result_traj_;

    double forward_point_distance_;

    std::vector<geometry_msgs::PoseStamped> global_plan_;

    boost::mutex configuration_mutex_;
    pcl::PointCloud<base_local_planner::MapGridCostPoint>* traj_cloud_;
    pcl_ros::Publisher<base_local_planner::MapGridCostPoint> traj_cloud_pub_;
    bool publish_cost_grid_pc_; ///< @brief Whether or not to build and publish a PointCloud
    bool publish_traj_pc_;

    double cheat_factor_;

    base_local_planner::MapGridVisualizer map_viz_; ///< @brief The map grid visualizer for outputting the potential field generated by the cost function

    // see constructor body for explanations
    AckermannTrajectoryGenerator generator_;
    base_local_planner::OscillationCostFunction oscillation_costs_;
    base_local_planner::ObstacleCostFunction obstacle_costs_;
    HeadingCostFunction heading_costs_;
    base_local_planner::MapGridCostFunction path_costs_;
    base_local_planner::MapGridCostFunction goal_costs_;

    AckermannTrajectorySearch scored_sampling_planner_;
};
#endif
