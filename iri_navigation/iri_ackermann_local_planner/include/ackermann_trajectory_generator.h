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
 * Author: TKruse
 *********************************************************************/

#ifndef ACKERMANN_TRAJECTORY_GENERATOR_H_
#define ACKERMANN_TRAJECTORY_GENERATOR_H_

#include <base_local_planner/trajectory_sample_generator.h>
#include <ackermann_planner_limits.h>
#include <Eigen/Core>

/**
 * generates trajectories based on equi-distant discretisation of the degrees of freedom.
 * This is supposed to be a simple and robust implementation of the TrajectorySampleGenerator
 * interface, more efficient implementations are thinkable.
 *
 * This can be used for both dwa and trajectory rollout approaches.
 * As an example, assuming these values:
 * sim_time = 1s, sim_period=200ms, dt = 200ms,
 * vsamples_x=5,
 * acc_limit_x = 1m/s^2, vel_x=0 (robot at rest, values just for easy calculations)
 * dwa_planner will sample max-x-velocities from 0m/s to 0.2m/s.
 * trajectory rollout approach will sample max-x-velocities 0m/s up to 1m/s
 * trajectory rollout approach does so respecting the acceleration limit, so it gradually increases velocity
 */
class AckermannTrajectoryGenerator: public base_local_planner::TrajectorySampleGenerator 
{
  public:

    AckermannTrajectoryGenerator();

    /**
     * @param pos current robot position
     * @param vel current robot velocity
     * @param limits Current velocity limits
     * @param vsamples: in how many samples to divide the given dimension
     * @param use_acceleration_limits: if true use physical model, else idealized robot model
     * @param discretize_by_time if true, the trajectory is split according in chunks of the same duration, else of same length
     */
    void initialise(
	const Eigen::Vector3f& pos,
	const Eigen::Vector3f& ackermann,
	const Eigen::Vector3f& goal,
	AckermannPlannerLimits* limits,
	const Eigen::Vector2f& vsamples,
	bool discretize_by_time = false);

    /**
     * This function is to be called only when parameters change
     *
     * @param sim_granularity granularity of collision detection
     * @param angular_sim_granularity angular granularity of collision detection
     * @param use_dwa whether to use DWA or trajectory rollout
     * @param sim_period distance between points in one trajectory
     */
    void set_parameters(
        double max_sim_time,
        double min_sim_time,
	double sim_granularity,
	double angular_sim_granularity,
	double sim_period = 0.0);

    /**
     * Whether this generator can create more trajectories
     */
    bool hasMoreTrajectories(void);

    /**
     * Whether this generator can create more trajectories
     */
    bool nextTrajectory(base_local_planner::Trajectory &traj);


    bool generate_trajectory(
	Eigen::Vector3f pos,
	Eigen::Vector3f vel,
	Eigen::Vector2f sample_target_vel,
	base_local_planner::Trajectory& traj);

    ~AckermannTrajectoryGenerator();
  protected:

    unsigned int next_sample_index_;
    // to store sample params of each sample between init and generation
    std::vector<Eigen::Vector2f> sample_params_;
    AckermannPlannerLimits* limits_;
    Eigen::Vector3f pos_;
    Eigen::Vector3f ackermann_;

    // whether velocity of trajectory changes over time or not
    bool discretize_by_time_;

    double max_sim_time_,min_sim_time_,sim_time_;
    double sim_granularity_, angular_sim_granularity_;
    double sim_period_; // only for dwa
};

#endif /* SIMPLE_TRAJECTORY_GENERATOR_H_ */
