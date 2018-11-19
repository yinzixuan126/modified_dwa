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

#include <ackermann_planner_ros.h>
#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(AckermannPlannerROS, nav_core::BaseLocalPlanner)

AckermannPlannerROS::AckermannPlannerROS() : initialized_(false),
  odom_helper_("odom",10), setup_(false) 
{
  patience_=1;
  this->stucked=false;
  this->first=true;
}

void AckermannPlannerROS::reconfigure_callback(iri_ackermann_local_planner::AckermannLocalPlannerConfig &config, uint32_t level) 
{
  if (setup_ && config.restore_defaults) 
  {
    config = default_config_;
    config.restore_defaults = false;
  }
  if ( ! setup_) 
  {
    default_config_ = config;
    setup_ = true;
  }

  // update generic local planner params
  AckermannPlannerLimits limits;
  limits.max_trans_vel = config.max_trans_vel;
  limits.min_trans_vel = config.min_trans_vel;
  limits.max_trans_acc = config.max_trans_acc;
  limits.max_steer_angle = config.max_steer_angle;
  limits.min_steer_angle = config.min_steer_angle;
  limits.max_steer_vel = config.max_steer_vel;
  limits.min_steer_vel = config.min_steer_vel;
  limits.max_steer_acc = config.max_steer_acc;
  // kinematic attributes
  limits.axis_distance = config.axis_distance;
  limits.wheel_distance = config.wheel_distance;
  limits.wheel_radius = config.wheel_radius;
  // general planner limits
  limits.xy_goal_tolerance = config.xy_goal_tolerance;
  limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
  limits.prune_plan = config.prune_plan;
  limits.trans_stopped_vel = config.trans_stopped_vel;
  limits.rot_stopped_vel = config.rot_stopped_vel;
  planner_util_.reconfigure_callback(limits, config.restore_defaults);
  this->last_cmds.resize(config.cmd_vel_avg);
  odom_helper_.set_average_samples(config.odom_avg);

  patience_=config.planner_patience;

  // update ackermann specific configuration
  dp_->reconfigure(config);
}

void AckermannPlannerROS::initialize(std::string name,tf::TransformListener* tf,costmap_2d::Costmap2DROS* costmap_ros) 
{
  if (! is_initialized()) 
  {
    ros::NodeHandle private_nh("~/" + name);
    g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
    l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ros_->getRobotPose(current_pose_);

    // make sure to update the costmap we'll use for this cycle
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

    planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

    //create the actual planner that we'll use.. it'll configure itself from the parameter server
    dp_ = boost::shared_ptr<AckermannPlanner>(new AckermannPlanner(name, &planner_util_));

    if( private_nh.getParam( "odom_topic", odom_topic_ ))
    {
      odom_helper_.set_odom_topic( odom_topic_ );
    }

    initialized_ = true;

    dsrv_ = new dynamic_reconfigure::Server<iri_ackermann_local_planner::AckermannLocalPlannerConfig>(private_nh);
    dynamic_reconfigure::Server<iri_ackermann_local_planner::AckermannLocalPlannerConfig>::CallbackType cb = boost::bind(&AckermannPlannerROS::reconfigure_callback, this, _1, _2);
    dsrv_->setCallback(cb);
    this->stucked=false;
    this->first=true;
  }
  else
  {
    ROS_WARN("This planner has already been initialized, doing nothing.");
  }
}

bool AckermannPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  if (! is_initialized()) 
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }
  //when we get a new plan, we also want to clear any latch we may have on goal tolerances
  // latchedStopRotateController_.resetLatching();
  this->stucked=false;
  this->first=true;
  for(unsigned int i=0;i<this->last_cmds.size();i++)
    this->last_cmds[i].linear.x=0.0;
  return dp_->set_plan(orig_global_plan);
}

void AckermannPlannerROS::publish_local_plan(std::vector<geometry_msgs::PoseStamped>& path) 
{
  base_local_planner::publishPlan(path, l_plan_pub_);
}


void AckermannPlannerROS::publish_global_plan(std::vector<geometry_msgs::PoseStamped>& path) 
{
  base_local_planner::publishPlan(path, g_plan_pub_);
}

bool AckermannPlannerROS::ackermann_compute_velocity_commands(tf::Stamped<tf::Pose> &global_pose, geometry_msgs::Twist& cmd_vel) 
{
  static int count=0;

  // dynamic window sampling approach to get useful velocity commands
  if(! is_initialized())
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  TAckermannState ackermann;
  AckermannPlannerLimits limits; 

  odom_helper_.get_ackermann_state(ackermann);
  limits=planner_util_.get_current_limits();
  // saturate speeds and angles
  if(ackermann.trans_speed>limits.max_trans_vel)
    ackermann.trans_speed=limits.max_trans_vel;
  else if(ackermann.trans_speed<limits.min_trans_vel)
    ackermann.trans_speed=limits.min_trans_vel;
  if(ackermann.steer_angle>limits.max_steer_angle)
    ackermann.steer_angle=limits.max_steer_angle;
  else if(ackermann.steer_angle<limits.min_steer_angle)
    ackermann.steer_angle=limits.min_steer_angle;
  if(ackermann.steer_speed>limits.max_steer_vel)
    ackermann.steer_speed=limits.max_steer_vel;
  else if(ackermann.steer_speed<limits.min_steer_vel)
    ackermann.steer_speed=limits.min_steer_vel;

  //compute what trajectory to drive along
  tf::Stamped<tf::Pose> drive_cmds;
  drive_cmds.frame_id_ = costmap_ros_->getBaseFrameID();

  // call with updated footprint
  base_local_planner::Trajectory path = dp_->find_best_path(global_pose, ackermann, drive_cmds, costmap_ros_->getRobotFootprint());
  //ROS_ERROR("Best: %.2f, %.2f, %.2f, %.2f", path.xv_, path.yv_, path.thetav_, path.cost_);

  //pass along drive commands
  cmd_vel.linear.x = drive_cmds.getOrigin().getX();
  cmd_vel.linear.y = drive_cmds.getOrigin().getY();
  cmd_vel.angular.z = tf::getYaw(drive_cmds.getRotation());

  //if we cannot move... tell someone
  std::vector<geometry_msgs::PoseStamped> local_plan;
  if(path.cost_ < 0) 
  {
    ROS_DEBUG_NAMED("ackermann_local_planner",
	"The ackermann local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
    local_plan.clear();
    publish_local_plan(local_plan);
    count++;
    if(count>patience_)
    {
      count=0;
      return false;
    }
    else
      return true;
  }
  else
    count=0;

  ROS_DEBUG_NAMED("ackermann_local_planner", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.", 
      cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

  // Fill out the local plan
  for(unsigned int i = 0; i < path.getPointsSize(); ++i) 
  {
    double p_x, p_y, p_th;
    path.getPoint(i, p_x, p_y, p_th);

    tf::Stamped<tf::Pose> p =
      tf::Stamped<tf::Pose>(tf::Pose(
	    tf::createQuaternionFromYaw(p_th),
	    tf::Point(p_x, p_y, 0.0)),
	  ros::Time::now(),
	  costmap_ros_->getGlobalFrameID());
    geometry_msgs::PoseStamped pose;
    tf::poseStampedTFToMsg(p, pose);
    local_plan.push_back(pose);
  }

  //publish information to the visualizer
  publish_local_plan(local_plan);
  return true;
}

bool AckermannPlannerROS::isGoalReached(void)
{
  nav_msgs::Odometry odom;
  AckermannPlannerLimits limits;

  if (! is_initialized()) {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }
  if ( ! costmap_ros_->getRobotPose(current_pose_)) {
    ROS_ERROR("Could not get robot pose");
    return false;
  }
  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  if ( ! planner_util_.get_local_plan(current_pose_, transformed_plan))
  {
    ROS_ERROR("Could not get local plan");
    return false;
  }
  odom_helper_.get_odom(odom);
  limits=planner_util_.get_current_limits();
  if(planner_util_.last_path())
  {
    if(base_local_planner::isGoalReached(*tf_,
                                         transformed_plan,
                                         *costmap_ros_->getCostmap(),
                                         costmap_ros_->getGlobalFrameID(),
                                         current_pose_,
                                         odom,
                                         limits.rot_stopped_vel,limits.trans_stopped_vel,
                                         limits.xy_goal_tolerance,limits.yaw_goal_tolerance))
    {
      ROS_INFO("Goal reached");
      return true;
    }
    else if(this->stucked)
      return true;
    else
      return false;
  } 
  else
    return false;
}

bool AckermannPlannerROS::is_initialized()
{
  return initialized_;
}

geometry_msgs::Twist AckermannPlannerROS::average_cmd_vel(geometry_msgs::Twist &new_cmd_vel)
{
  static int current_index=0;
  unsigned int i=0;
  geometry_msgs::Twist avg_cmd_vel;

  this->last_cmds[current_index].linear.x=new_cmd_vel.linear.x;
  current_index=(current_index+1)%this->last_cmds.size();
  avg_cmd_vel.linear.x=0;
  avg_cmd_vel.linear.y=0;
  avg_cmd_vel.angular.z=0;

  for(i=0;i<this->last_cmds.size();i++)
    avg_cmd_vel.linear.x+=this->last_cmds[i].linear.x;

  avg_cmd_vel.linear.x/=this->last_cmds.size();
  avg_cmd_vel.linear.y=new_cmd_vel.linear.y;
  avg_cmd_vel.angular.z=new_cmd_vel.angular.z;

  return avg_cmd_vel;
}

bool AckermannPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) 
{
  geometry_msgs::Twist tmp_cmd_vel;
  nav_msgs::Odometry odom;
  AckermannPlannerLimits limits;
  tf::Stamped<tf::Pose> goal_pose;
  static int stucked_count=0;
  static int replan_count=0;
  static bool new_segment=false;

  // dispatches to either ackermann sampling control or stop and rotate control, depending on whether we have been close enough to goal
  if ( ! costmap_ros_->getRobotPose(current_pose_)) 
  {
    ROS_ERROR("Could not get robot pose");
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    this->stucked=false;
    new_segment=false;
    replan_count=0;
    stucked_count=0;
    return false;
  }
  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  if ( ! planner_util_.get_local_plan(current_pose_, transformed_plan)) 
  {
    ROS_ERROR("Could not get local plan");
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    this->stucked=false;
    new_segment=false;
    replan_count=0;
    stucked_count=0;
    return false;
  }

  //if the global plan passed in is empty... we won't do anything
  if(transformed_plan.empty()) 
  {
    ROS_WARN_NAMED("ackermann_local_planner", "Received an empty transformed plan.");
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    this->stucked=false;
    new_segment=false;
    replan_count=0;
    stucked_count=0;
    return false;
  }
  ROS_DEBUG_NAMED("ackermann_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

  // update plan in ackermann_planner even if we just stop and rotate, to allow checkTrajectory
  dp_->update_plan_and_local_costs(current_pose_, transformed_plan);

  odom_helper_.get_odom(odom);
  limits=planner_util_.get_current_limits();
  if(this->stucked || base_local_planner::isGoalReached(*tf_,
                                                        transformed_plan,
                                                        *costmap_ros_->getCostmap(),
                                                        costmap_ros_->getGlobalFrameID(),
                                                        current_pose_,
                                                        odom,
                                                        limits.rot_stopped_vel,limits.trans_stopped_vel,
                                                        limits.xy_goal_tolerance,limits.yaw_goal_tolerance))
  {
    if(planner_util_.set_next_path())
    {
      this->stucked=false;
      new_segment=true;
      bool isOk = ackermann_compute_velocity_commands(current_pose_, tmp_cmd_vel);
      cmd_vel=this->average_cmd_vel(tmp_cmd_vel);
      if (isOk) 
        publish_global_plan(transformed_plan);
      else 
      {
        ROS_WARN_NAMED("ackermann_local_planner", "Ackermann planner failed to produce path.");
        std::vector<geometry_msgs::PoseStamped> empty_plan;
        publish_global_plan(empty_plan);
      }
      return isOk;
    }
    else
    {
      std::vector<geometry_msgs::PoseStamped> local_plan;
      std::vector<geometry_msgs::PoseStamped> transformed_plan;
      publish_global_plan(transformed_plan);
      publish_local_plan(local_plan);
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;
      this->stucked=false;
      new_segment=false;
      replan_count=0;
      stucked_count=0;
    }
  }
  else
  {
    bool isOk = ackermann_compute_velocity_commands(current_pose_, tmp_cmd_vel);
    cmd_vel=this->average_cmd_vel(tmp_cmd_vel);
    // check wether the robot get stucked or not
    if(new_segment || this->first)
    {
      TAckermannState ackermann;
      odom_helper_.get_ackermann_state(ackermann);
      // steer the car without moving if the steering angle is big
      
      if(fabs(ackermann.steer_angle-cmd_vel.angular.z)>0.05)
      {
        ROS_WARN("Setting forward speed to 0 (%f,%f)",ackermann.steer_angle,cmd_vel.angular.z);
        cmd_vel.linear.x=0.0;
      }
      else
      {
        if(this->first) this->first=false;
        if(new_segment) new_segment=false;
      }
    }
    else if(fabs(cmd_vel.linear.x)<0.015)
    {
       // get the position of the goal
       base_local_planner::getGoalPose(*tf_,
                                       transformed_plan,
                                       costmap_ros_->getGlobalFrameID(),
                                       goal_pose);
       // get the distance to the goal
       double dist=base_local_planner::getGoalPositionDistance(current_pose_,goal_pose.getOrigin().getX(),goal_pose.getOrigin().getY());
       if(dist>limits.xy_goal_tolerance && dist<2*limits.xy_goal_tolerance)
       {
         stucked_count++;
         if(stucked_count>10)
         {
           this->stucked=true;
           ROS_INFO("Robot is stucked, jumping to the next segment");
         }
       }
       else
       {
         replan_count++;
         if(replan_count>10)
         {
           ROS_INFO("Replan because robot can not move");
           cmd_vel.linear.x = 0.0;
           cmd_vel.linear.y = 0.0;
           cmd_vel.angular.z = 0.0;
           this->stucked=false;
           new_segment=false;
           replan_count=0;
           stucked_count=0;
           return false;
         }
       }
    }
    else
    {
      stucked_count=0;
      replan_count=0;
    }
    if (isOk) 
      publish_global_plan(transformed_plan);
    else 
    {
      ROS_WARN_NAMED("ackermann_local_planner", "Ackermann planner failed to produce path.");
      std::vector<geometry_msgs::PoseStamped> empty_plan;
      publish_global_plan(empty_plan);
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;
      this->stucked=false;
      new_segment=false;
      replan_count=0;
      stucked_count=0;
    }
    
    return isOk;
  }
}

AckermannPlannerROS::~AckermannPlannerROS()
{
  //make sure to clean things up
  delete dsrv_;
}

