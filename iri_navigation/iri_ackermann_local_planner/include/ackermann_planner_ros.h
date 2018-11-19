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
#ifndef ACKERMANN_PLANNER_ROS_H_
#define ACKERMANN_PLANNER_ROS_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <iri_ackermann_local_planner/AckermannLocalPlannerConfig.h>

#include <angles/angles.h>

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>

#include <ackermann_odom_helper.h>
#include <ackermann_planner_util.h>

#include <ackermann_planner.h>

/**
 * @class AckermannPlannerROS
 * @brief ROS Wrapper for the DWAPlanner that adheres to the
 * BaseLocalPlanner interface and can be used as a plugin for move_base.
 */
class AckermannPlannerROS : public nav_core::BaseLocalPlanner 
{
  public:
    /**
     * @brief  Constructor for AckermannPlannerROS wrapper
     */
    AckermannPlannerROS();

    /**
     * @brief  Constructs the ros wrapper
     * @param name The name to give this instance of the trajectory planner
     * @param tf A pointer to a transform listener
     * @param costmap The cost map to use for assigning costs to trajectories
     */
    void initialize(std::string name, tf::TransformListener* tf,costmap_2d::Costmap2DROS* costmap_ros);

    /**
     * @brief  Given the current position, orientation, and velocity of the robot,
     * compute velocity commands to send to the base
     * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
     * @return True if a valid trajectory was found, false otherwise
     */
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    /**
     * @brief  Given the current position, orientation, and velocity of the robot,
     * compute velocity commands to send to the base, using dynamic window approach
     * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
     * @return True if a valid trajectory was found, false otherwise
     */
    bool ackermann_compute_velocity_commands(tf::Stamped<tf::Pose>& global_pose, geometry_msgs::Twist& cmd_vel);

    /**
     * @brief  Set the plan that the controller is following
     * @param orig_global_plan The plan to pass to the controller
     * @return True if the plan was updated successfully, false otherwise
     */
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool isGoalReached();

    bool is_initialized();

    geometry_msgs::Twist average_cmd_vel(geometry_msgs::Twist &new_cmd_vel);

    /**
     * @brief  Destructor for the wrapper
     */
    ~AckermannPlannerROS();

  private:
    std::vector<geometry_msgs::Twist> last_cmds;
    /**
     * @brief Callback to update the local planner's parameters based on dynamic reconfigure
     */
    void reconfigure_callback(iri_ackermann_local_planner::AckermannLocalPlannerConfig &config, uint32_t level);

    void publish_local_plan(std::vector<geometry_msgs::PoseStamped>& path);

    void publish_global_plan(std::vector<geometry_msgs::PoseStamped>& path);

    tf::TransformListener* tf_; ///< @brief Used for transforming point clouds

    // for visualisation, publishers of global and local plan
    ros::Publisher g_plan_pub_, l_plan_pub_;

    AckermannPlannerUtil planner_util_;

    boost::shared_ptr<AckermannPlanner> dp_; ///< @brief The trajectory controller

    costmap_2d::Costmap2DROS* costmap_ros_;

    dynamic_reconfigure::Server<iri_ackermann_local_planner::AckermannLocalPlannerConfig> *dsrv_;
    iri_ackermann_local_planner::AckermannLocalPlannerConfig default_config_;
    bool setup_;
    tf::Stamped<tf::Pose> current_pose_;
    bool initialized_;
    int patience_;

    AckermannOdomHelper odom_helper_;
    std::string odom_topic_;

    bool stucked;
    bool first;
};
#endif
