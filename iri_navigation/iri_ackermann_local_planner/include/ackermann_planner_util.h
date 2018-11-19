/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef ACKERMANN_PLANNER_UTIL_H_
#define ACKERMANN_PLANNER_UTIL_H_

#include <nav_core/base_local_planner.h>

#include <boost/thread.hpp>

#include <costmap_2d/costmap_2d.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <ackermann_planner_limits.h>


/**
 * @class AckermannPlannerUtil
 * @brief Helper class implementing infrastructure code many local planner implementations may need.
 */
class AckermannPlannerUtil 
{
  private:
    // things we get from move_base
    std::string name_;
    std::string global_frame_;

    costmap_2d::Costmap2D* costmap_;
    tf::TransformListener* tf_;

    std::vector<geometry_msgs::PoseStamped> global_plan_;

    boost::mutex limits_configuration_mutex_;
    bool setup_;
    AckermannPlannerLimits default_limits_;
    AckermannPlannerLimits limits_;
    bool initialized_;

    std::vector < std::vector<geometry_msgs::PoseStamped> > subPathList;
    std::vector < geometry_msgs::PoseStamped > subPath;
    unsigned int subPathIndex;

  public:

    AckermannPlannerUtil();
    /**
     * @brief  Callback to update the local planner's parameters
     */
    void reconfigure_callback(AckermannPlannerLimits &config, bool restore_defaults);

    void initialize(tf::TransformListener* tf,costmap_2d::Costmap2D* costmap,std::string global_frame);

    bool get_goal(tf::Stamped<tf::Pose>& goal_pose);

    bool set_plan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool set_next_path(void);
   
    bool last_path(void);

    bool get_local_plan(tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& transformed_plan);

    costmap_2d::Costmap2D* get_costmap();

    AckermannPlannerLimits get_current_limits();

    std::string get_global_frame(void);

    ~AckermannPlannerUtil();
};

#endif 
