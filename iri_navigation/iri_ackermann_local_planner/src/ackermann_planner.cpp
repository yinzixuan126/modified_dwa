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
#include <ackermann_planner.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/map_grid_cost_point.h>
#include <cmath>

//for computing path distance
#include <queue>

#include <angles/angles.h>

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>

void AckermannPlanner::reconfigure(iri_ackermann_local_planner::AckermannLocalPlannerConfig &config)
{
  boost::mutex::scoped_lock l(configuration_mutex_);

  generator_.set_parameters(
      config.max_sim_time,
      config.min_sim_time,
      config.sim_granularity,
      config.angular_sim_granularity,
      sim_period_);

  double resolution = planner_util_->get_costmap()->getResolution();
  pdist_scale_ = config.path_distance_bias;
  // pdistscale used for both path and alignment, set  forward_point_distance to zero to discard alignment
  path_costs_.setScale(resolution * pdist_scale_ * 0.5);
  hdiff_scale_=config.hdiff_scale;
  heading_costs_.setScale(hdiff_scale_);

  gdist_scale_ = config.goal_distance_bias;
  goal_costs_.setScale(resolution * gdist_scale_ * 0.5);

  occdist_scale_ = config.occdist_scale;
  obstacle_costs_.setScale(resolution * occdist_scale_);

  stop_time_buffer_ = config.stop_time_buffer;
  oscillation_costs_.setOscillationResetDist(config.oscillation_reset_dist, config.oscillation_reset_angle);
  forward_point_distance_ = config.forward_point_distance;
  heading_costs_.set_num_points(config.heading_points);

  // obstacle costs can vary due to scaling footprint feature
  obstacle_costs_.setParams(config.max_trans_vel, config.max_scaling_factor, config.scaling_speed);

  int trans_vel_samp, steer_angle_samp;
  trans_vel_samp = config.trans_vel_samples;
  steer_angle_samp = config.steer_angle_samples;

  if (trans_vel_samp <= 0) 
  {
    ROS_WARN("You've specified that you don't want any samples in the translational dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
    trans_vel_samp = 1;
    config.trans_vel_samples = trans_vel_samp;
  }

  if (steer_angle_samp <= 0) 
  {
    ROS_WARN("You've specified that you don't want any samples in the steering dimension. We'll at least assume that you want to sample one value... so we're going to set vy_samples to 1 instead");
    steer_angle_samp = 1;
    config.steer_angle_samples = steer_angle_samp;
  }

  vsamples_[0] = trans_vel_samp;
  vsamples_[1] = steer_angle_samp;
}

AckermannPlanner::AckermannPlanner(std::string name,AckermannPlannerUtil *planner_util) :
  planner_util_(planner_util),
  obstacle_costs_(planner_util->get_costmap()),
  path_costs_(planner_util->get_costmap()),
  goal_costs_(planner_util->get_costmap(), 0.0, 0.0, true)
{
  ros::NodeHandle private_nh("~/" + name);

  //Assuming this planner is being run within the navigation stack, we can
  //just do an upward search for the frequency at which its being run. This
  //also allows the frequency to be overwritten locally.
  std::string controller_frequency_param_name;
  if(!private_nh.searchParam("controller_frequency", controller_frequency_param_name)) 
  {
    sim_period_ = 0.05;
  } 
  else 
  {
    double controller_frequency = 0;
    private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
    if(controller_frequency > 0) 
    {
      sim_period_ = 1.0 / controller_frequency;
    }
    else 
    {
      ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
      sim_period_ = 0.05;
    }
  }

  oscillation_costs_.resetOscillationFlags();

  bool sum_scores;
  private_nh.param("sum_scores", sum_scores, false);
  obstacle_costs_.setSumScores(sum_scores);

  private_nh.param("publish_cost_grid_pc", publish_cost_grid_pc_, false);
  map_viz_.initialize(name, planner_util->get_global_frame(), boost::bind(&AckermannPlanner::get_cell_costs, this, _1, _2, _3, _4, _5, _6));

  std::string frame_id;
  private_nh.param("global_frame_id", frame_id, std::string("odom"));

  traj_cloud_ = new pcl::PointCloud<base_local_planner::MapGridCostPoint>;
  traj_cloud_->header.frame_id = frame_id;
  traj_cloud_pub_.advertise(private_nh, "trajectory_cloud", 1);
  private_nh.param("publish_traj_pc", publish_traj_pc_, false);

  // set up all the cost functions that will be applied in order
  // (any function returning negative values will abort scoring, so the order can improve performance)
  std::vector<base_local_planner::TrajectoryCostFunction*> critics;
  critics.push_back(&oscillation_costs_); // discards oscillating motions (assisgns cost -1)
  critics.push_back(&obstacle_costs_); // discards trajectories that move into obstacles
  critics.push_back(&path_costs_); // prefers trajectories on global path
  critics.push_back(&goal_costs_); // prefers trajectories that go towards (local) goal, based on wave propagation
  critics.push_back(&heading_costs_); // prefers trajectories that go towards (local) goal, based on wave propagation

  // trajectory generators
  std::vector<base_local_planner::TrajectorySampleGenerator*> generator_list;
  generator_list.push_back(&generator_);

  scored_sampling_planner_ = AckermannTrajectorySearch(generator_list, critics);

  private_nh.param("cheat_factor", cheat_factor_, 1.0);
}

// used for visualization only, total_costs are not really total costs
bool AckermannPlanner::get_cell_costs(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {

  path_cost = path_costs_.getCellCosts(cx, cy);
  goal_cost = goal_costs_.getCellCosts(cx, cy);
  occ_cost = planner_util_->get_costmap()->getCost(cx, cy);
  if (path_cost == path_costs_.obstacleCosts() ||
      path_cost == path_costs_.unreachableCellCosts() ||
      occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    return false;
  }

  double resolution = planner_util_->get_costmap()->getResolution();
  total_cost =
    pdist_scale_ * resolution * path_cost +
    gdist_scale_ * resolution * goal_cost +
    occdist_scale_ * occ_cost;
  return true;
}

bool AckermannPlanner::set_plan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  oscillation_costs_.resetOscillationFlags();
  return planner_util_->set_plan(orig_global_plan);
}

/**
 * This function is used when other strategies are to be applied,
 * but the cost functions for obstacles are to be reused.
 */
bool AckermannPlanner::check_trajectory(Eigen::Vector3f pos,Eigen::Vector3f ackermann,Eigen::Vector2f samples)
{
  oscillation_costs_.resetOscillationFlags();
  base_local_planner::Trajectory traj;
  geometry_msgs::PoseStamped goal_pose = global_plan_.back();
  Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf::getYaw(goal_pose.pose.orientation));
  AckermannPlannerLimits limits = planner_util_->get_current_limits();
  generator_.initialise(pos,
      ackermann,
      goal,
      &limits,
      vsamples_);
  generator_.generate_trajectory(pos, ackermann, samples, traj);
  double cost = scored_sampling_planner_.scoreTrajectory(traj, -1);
  //if the trajectory is a legal one... the check passes
  if(cost >= 0) 
    return true;
  ROS_WARN("Invalid Trajectory %f, %f, cost: %f", samples[0], samples[1], cost);

  //otherwise the check fails
  return false;
}

void AckermannPlanner::update_plan_and_local_costs(tf::Stamped<tf::Pose> global_pose,const std::vector<geometry_msgs::PoseStamped>& new_plan) 
{
  global_plan_.resize(new_plan.size());
  for (unsigned int i = 0; i < new_plan.size(); ++i) 
    global_plan_[i] = new_plan[i];

  // costs for going away from path
  path_costs_.setTargetPoses(global_plan_);

  // costs for not going towards the local goal as much as possible
  goal_costs_.setTargetPoses(global_plan_);

  heading_costs_.set_global_plan(global_plan_);

  // alignment costs
  geometry_msgs::PoseStamped goal_pose = global_plan_.back();

}

/*
 * given the current state of the robot, find a good trajectory
 */
base_local_planner::Trajectory AckermannPlanner::find_best_path(tf::Stamped<tf::Pose> global_pose,TAckermannState ack_state,tf::Stamped<tf::Pose>& drive_velocities,std::vector<geometry_msgs::Point> footprint_spec) 
{
  obstacle_costs_.setFootprint(footprint_spec);

  //make sure that our configuration doesn't change mid-run
  boost::mutex::scoped_lock l(configuration_mutex_);

  Eigen::Vector3f pos(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), tf::getYaw(global_pose.getRotation()));
  Eigen::Vector3f ackermann(ack_state.trans_speed,ack_state.steer_angle,ack_state.steer_speed);
  geometry_msgs::PoseStamped goal_pose = global_plan_.back();
  Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf::getYaw(goal_pose.pose.orientation));
  AckermannPlannerLimits limits = planner_util_->get_current_limits();

  // prepare cost functions and generators for this run
  generator_.initialise(pos,
      ackermann,
      goal,
      &limits,
      vsamples_);

  result_traj_.cost_ = -7;
  // find best trajectory by sampling and scoring the samples
  std::vector<base_local_planner::Trajectory> all_explored;
  scored_sampling_planner_.findBestTrajectory(result_traj_, &all_explored);

  if(publish_traj_pc_)
  {
    base_local_planner::MapGridCostPoint pt;
    traj_cloud_->points.clear();
    traj_cloud_->width = 0;
    traj_cloud_->height = 0;
    std_msgs::Header header;
    pcl_conversions::fromPCL(traj_cloud_->header, header);
    header.stamp = ros::Time::now();
    traj_cloud_->header = pcl_conversions::toPCL(header);
    for(std::vector<base_local_planner::Trajectory>::iterator t=all_explored.begin(); t != all_explored.end(); ++t)
    {
      if(t->cost_<0)
	continue;
      // Fill out the plan
      for(unsigned int i = 0; i < t->getPointsSize(); ++i) 
      {
	double p_x, p_y, p_th;
	t->getPoint(i, p_x, p_y, p_th);
	pt.x=p_x;
	pt.y=p_y;
	pt.z=0;
	pt.path_cost=p_th;
	pt.total_cost=t->cost_;
	traj_cloud_->push_back(pt);
      }
    }
    traj_cloud_pub_.publish(*traj_cloud_);
  }

  // verbose publishing of point clouds
  if (publish_cost_grid_pc_) 
  {
    //we'll publish the visualization of the costs to rviz before returning our best trajectory
    map_viz_.publishCostCloud(planner_util_->get_costmap());
  }

  // debrief stateful scoring functions
  oscillation_costs_.updateOscillationFlags(pos, &result_traj_, planner_util_->get_current_limits().min_trans_vel);

  //if we don't have a legal trajectory, we'll just command zero
  if (result_traj_.cost_ < 0) 
  {
    drive_velocities.setIdentity();
  } 
  else 
  {
    tf::Vector3 start(result_traj_.xv_, result_traj_.yv_, 0);
    drive_velocities.setOrigin(start);
    tf::Matrix3x3 matrix;
    matrix.setRotation(tf::createQuaternionFromYaw(result_traj_.thetav_));
    drive_velocities.setBasis(matrix);
  }

  return result_traj_;
}

AckermannPlanner::~AckermannPlanner()
{
  if(traj_cloud_)
    delete traj_cloud_;
}
