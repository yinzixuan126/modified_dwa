/***********************************************************
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 ***********************************************************/


#ifndef _ACKERMANN_PLANNER_LIMITS_H
#define _ACKERMANN_PLANNER_LIMITS_H

#include <Eigen/Core>

class AckermannPlannerLimits
{
  public:
    // translational limits
    double max_trans_vel;
    double min_trans_vel;
    double max_trans_acc;
    // steering limits
    double max_steer_angle;
    double min_steer_angle;
    double max_steer_vel;
    double min_steer_vel;
    double max_steer_acc;
    // kinematic attributes
    double axis_distance;
    double wheel_distance;
    double wheel_radius; 
    // general planner limits
    bool prune_plan;
    double xy_goal_tolerance;
    double yaw_goal_tolerance;
    double trans_stopped_vel;
    double rot_stopped_vel;
    bool restore_defaults;

    AckermannPlannerLimits() {}

    AckermannPlannerLimits(
      // translational limits
      double max_trans_vel,
      double min_trans_vel,
      double max_trans_acc,
      // steering limits
      double max_steer_angle,
      double min_steer_angle,
      double max_steer_vel,
      double min_steer_vel,
      double max_steer_acc,
      // kinematic attributes
      double axis_distance,
      double wheel_distance,
      double wheel_radius, 
      // general planner limits
      double xy_goal_tolerance,
      double yaw_goal_tolerance,
      bool prune_plan=true,
      double trans_stopped_vel=0.1,
      double rot_stopped_vel=0.1):
      max_trans_vel(max_trans_vel),
      min_trans_vel(min_trans_vel),
      max_trans_acc(max_trans_acc),
      max_steer_angle(max_steer_angle),
      min_steer_angle(min_steer_angle),
      max_steer_vel(max_steer_vel),
      min_steer_vel(min_steer_vel),
      max_steer_acc(max_steer_acc),
      axis_distance(axis_distance),
      wheel_distance(wheel_distance),
      wheel_radius(wheel_radius),
      prune_plan(prune_plan),
      xy_goal_tolerance(xy_goal_tolerance),
      yaw_goal_tolerance(yaw_goal_tolerance),
      trans_stopped_vel(trans_stopped_vel),
      rot_stopped_vel(rot_stopped_vel)
    {
    }

    ~AckermannPlannerLimits()
    {
    }
};

#endif // __LOCALPLANNERLIMITS_H__
