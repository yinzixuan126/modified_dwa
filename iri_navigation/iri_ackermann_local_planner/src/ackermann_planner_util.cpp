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

#include <ackermann_planner_util.h>

#include <base_local_planner/goal_functions.h>

AckermannPlannerUtil::AckermannPlannerUtil():initialized_(false)
{

}

void AckermannPlannerUtil::initialize(tf::TransformListener* tf, costmap_2d::Costmap2D* costmap, std::string global_frame) 
{
  if(!initialized_) 
  {
    tf_ = tf;
    costmap_ = costmap;
    global_frame_ = global_frame;
    initialized_ = true;
  }
  else
  {
    ROS_WARN("Planner utils have already been initialized, doing nothing.");
  }
}

void AckermannPlannerUtil::reconfigure_callback(AckermannPlannerLimits &config, bool restore_defaults)
{
  if(setup_ && restore_defaults) 
    config = default_limits_;

  if(!setup_) 
  {
    default_limits_ = config;
    setup_ = true;
  }
  boost::mutex::scoped_lock l(limits_configuration_mutex_);
  limits_ = AckermannPlannerLimits(config);
}

costmap_2d::Costmap2D* AckermannPlannerUtil::get_costmap() 
{
  return costmap_;
}

AckermannPlannerLimits AckermannPlannerUtil::get_current_limits() 
{
  boost::mutex::scoped_lock l(limits_configuration_mutex_);
  return limits_;
}

std::string AckermannPlannerUtil::get_global_frame(void)
{
  return global_frame_;
}

bool AckermannPlannerUtil::get_goal(tf::Stamped<tf::Pose>& goal_pose) 
{
  //we assume the global goal is the last point in the global plan
  return base_local_planner::getGoalPose(*tf_,global_plan_,global_frame_,goal_pose);
}

bool AckermannPlannerUtil::set_plan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  if(!initialized_)
  {
    ROS_ERROR("Planner utils have not been initialized, please call initialize() first");
    return false;
  }
  ////divide plan by manuveurs
  subPathList.clear();
  subPath.clear();
  subPathIndex      = 0;
  double pathLength = 0;
  for(unsigned int i = 0; i < orig_global_plan.size(); ++i)
  {
    if(i>1 && i<orig_global_plan.size())
    {
      double x0 = orig_global_plan[i  ].pose.position.x;
      double x1 = orig_global_plan[i-1].pose.position.x;
      double x2 = orig_global_plan[i-2].pose.position.x;
      double y0 = orig_global_plan[i  ].pose.position.y;
      double y1 = orig_global_plan[i-1].pose.position.y;
      double y2 = orig_global_plan[i-2].pose.position.y;
      double angle=((x0-x1)*(x1-x2)+(y0-y1)*(y1-y2))/(std::sqrt(std::pow(x0-x1,2)+std::pow(y0-y1,2))*std::sqrt(std::pow(x1-x2,2)+std::pow(y1-y2,2)));
      if(angle<-1.0)
        angle=-1.0;
      else if(angle>1.0)
        angle=1.0;
      angle=std::acos(angle);
      pathLength+= std::sqrt(std::pow(x0-x1,2)+std::pow(y0-y1,2));
      if(fabs(angle)>1.57) //if changes of direction detected
      {
        if(pathLength>1.0)
        {
          ROS_INFO("TrajectoryPlannerROS::setPlan: subPath split at i=%d, angle=%f, length=%f", i, angle, pathLength);
          subPathList.push_back(subPath);
        }
        else //ignored subpaths shorter than 1.0m
        {
          ROS_INFO("TrajectoryPlannerROS::setPlan: subPath split at i=%d, angle=%f, Ignored by length=%f", i, angle, pathLength);
        }
        subPath.clear();
        pathLength=0.0;
      }
    }
    subPath.push_back(orig_global_plan[i]);
  }
  subPathList.push_back(subPath);
  ROS_INFO("TrajectoryPlannerROS::setPlan: subPath last length=%f", pathLength);
  ROS_INFO("TrajectoryPlannerROS::setPlan: Global plan (%lu points) split in %lu paths", orig_global_plan.size(), subPathList.size());

  //reset the global plan
  global_plan_.clear();

  global_plan_ = subPathList[subPathIndex];

  return true;
}

bool AckermannPlannerUtil::set_next_path(void)
{
  ////check if there are manuveurs remaining
  if(subPathIndex < subPathList.size()-1)
  {
    subPathIndex++;
    global_plan_.clear();
    global_plan_ = subPathList[subPathIndex];
    return true;
  }
  else
    return false;
  ////
}

bool AckermannPlannerUtil::last_path(void)
{
  if(subPathIndex==subPathList.size()-1)
    return true;
  else
    return false;
}

bool AckermannPlannerUtil::get_local_plan(tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& transformed_plan) 
{
  global_pose.stamp_-=ros::Duration(1.0);

  //get the global plan in our frame
  if(!base_local_planner::transformGlobalPlan(*tf_,global_plan_,global_pose,*costmap_,global_frame_,transformed_plan)) {
    ROS_WARN("Could not transform the global plan to the frame of the controller");
    return false;
  }

  //now we'll prune the plan based on the position of the robot
  if(limits_.prune_plan) 
    base_local_planner::prunePlan(global_pose, transformed_plan, global_plan_);

  return true;
}

AckermannPlannerUtil::~AckermannPlannerUtil()
{

}
