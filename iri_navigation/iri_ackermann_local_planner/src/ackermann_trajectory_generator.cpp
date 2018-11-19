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

#include <ackermann_trajectory_generator.h>

#include <cmath>
#include <iostream>
#include <algorithm> 

#include <base_local_planner/velocity_iterator.h>

#include <ros/console.h>

AckermannTrajectoryGenerator::AckermannTrajectoryGenerator()
{
  limits_=NULL;
}

void AckermannTrajectoryGenerator::initialise(
  const Eigen::Vector3f& pos,
    const Eigen::Vector3f& ackermann,//[0] -> trans_vel, [1]-> steer_angle, [2]-> steer_vel
    const Eigen::Vector3f& goal,
    AckermannPlannerLimits* limits,
    const Eigen::Vector2f& vsamples,//trans_vel samples & steer_angle samples
    bool discretize_by_time) 
{
  double steer_angle,steer_vel,trans_vel;
  double max_steer_vel,min_steer_vel;
  double max_steer_angle,min_steer_angle;
  double max_trans_vel,min_trans_vel;
  double T4=0.0;
  /*
   * We actually generate all velocity sample vectors here, from which to generate trajectories later on
   */
  pos_ = pos;
  limits_ = limits;
  ackermann_ = ackermann;
  next_sample_index_ = 0;
  sample_params_.clear();
  // saturate the current ackermann state if necessary
/*  if(ackermann[0]>limits->max_trans_vel)
    trans_vel=limits->max_trans_vel;
  else if (ackermann[0]<limits->min_trans_vel)
    trans_vel=limits->min_trans_vel;
  else*/
  trans_vel=ackermann[0];
/*  if(ackermann[1]>limits->max_steer_angle)
    steer_angle=limits->max_steer_angle;
  else if(ackermann[1]<limits->min_steer_angle)
    steer_angle=limits->min_steer_angle;
  else */
  steer_angle=ackermann[1];
/*  if(ackermann[2]>limits->max_steer_vel)
    steer_vel=limits->max_steer_vel;
  else if(ackermann[2]<limits->min_steer_vel)
    steer_vel=limits->min_steer_vel;
  else*/
  steer_vel=ackermann[2];
  // compute the simulation time
  double dist=sqrt((goal[0]-pos[0])*(goal[0]-pos[0])+(goal[1]-pos[1])*(goal[1]-pos[1]));
  sim_time_=dist/limits->max_trans_vel+limits->max_trans_vel/limits->max_trans_acc;
  if(sim_time_>max_sim_time_)
    sim_time_=max_sim_time_;
  else if(sim_time_<min_sim_time_)
    sim_time_=min_sim_time_;
  // compute the trajectory times and windows
  if(steer_vel>=0)
  {
    max_steer_vel=(steer_vel+limits->max_steer_acc*sim_time_)/2.0;
    if(max_steer_vel>=limits->max_steer_vel)
    {
      max_steer_vel=limits->max_steer_vel;  
      T4=sim_time_-(2.0*max_steer_vel-steer_vel)/limits->max_steer_acc;
      max_steer_angle=max_steer_vel*max_steer_vel/limits->max_steer_acc-steer_vel*steer_vel/(2.0*limits->max_steer_acc)+max_steer_vel*T4+steer_angle;
      if(max_steer_angle>limits->max_steer_angle) 
      {
       max_steer_angle=limits->max_steer_angle;
       T4=(limits->max_steer_angle-steer_angle-max_steer_vel*max_steer_vel/limits->max_steer_acc+steer_vel*steer_vel/(2.0*limits->max_steer_acc))/max_steer_vel;
       if(T4<0)
       {
         T4=0;
         max_steer_vel=sqrt(steer_vel*steer_vel/2.0+limits->max_steer_acc*(limits->max_steer_angle-steer_angle));
       }
     }
   }
   else
   { 
    max_steer_angle=max_steer_vel*max_steer_vel/limits->max_steer_acc-steer_vel*steer_vel/(2.0*limits->max_steer_acc)+steer_angle;
    T4=0;
    if(max_steer_angle>limits->max_steer_angle)
    {
     max_steer_angle=limits->max_steer_angle;
     max_steer_vel=sqrt(steer_vel*steer_vel/2.0+limits->max_steer_acc*(limits->max_steer_angle-steer_angle));
   }
 }
 min_steer_vel=(steer_vel-limits->max_steer_acc*sim_time_)/2.0;
 if(min_steer_vel<=limits->min_steer_vel)
 {
  min_steer_vel=limits->min_steer_vel;
  T4=sim_time_+(2.0*min_steer_vel-steer_vel)/limits->max_steer_acc;
  min_steer_angle=steer_vel*steer_vel/(2.0*limits->max_steer_acc)-min_steer_vel*min_steer_vel/limits->max_steer_acc+min_steer_vel*T4+steer_angle;
  if(min_steer_angle<limits->min_steer_angle)
  {
   min_steer_angle=limits->min_steer_angle;
   T4=(limits->min_steer_angle-steer_angle+min_steer_vel*min_steer_vel/limits->max_steer_acc-steer_vel*steer_vel/(2.0*limits->max_steer_acc))/min_steer_vel;
   if(T4<0)
   {
     T4=0;
     min_steer_vel=-sqrt(steer_vel*steer_vel/2.0-limits->max_steer_acc*(limits->min_steer_angle-steer_angle));
   }
 }
}
else
{
  min_steer_angle=steer_vel*steer_vel/(2.0*limits->max_steer_acc)-min_steer_vel*min_steer_vel/limits->max_steer_acc+steer_angle;
  T4=0;
  if(min_steer_angle<limits->min_steer_angle)
  {
   min_steer_angle=limits->min_steer_angle;
   min_steer_vel=-sqrt(steer_vel*steer_vel/2.0-limits->max_steer_acc*(limits->min_steer_angle-steer_angle)); 
 }
}
}
else
{
  max_steer_vel=(steer_vel+limits->max_steer_acc*sim_time_)/2.0;
  if(max_steer_vel>=limits->max_steer_vel)
  {
    max_steer_vel=limits->max_steer_vel;
    T4=sim_time_-(2.0*max_steer_vel-steer_vel)/limits->max_steer_acc;
    max_steer_angle=-steer_vel*steer_vel/(2.0*limits->max_steer_acc)+max_steer_vel*max_steer_vel/limits->max_steer_acc+max_steer_vel*T4+steer_angle;
    if(max_steer_angle>limits->max_steer_angle)
    {
     max_steer_angle=limits->max_steer_angle;
     T4=(limits->max_steer_angle-steer_angle-max_steer_vel*max_steer_vel/limits->max_steer_acc+steer_vel*steer_vel/(2.0*limits->max_steer_acc))/max_steer_vel;
     if(T4<0)
     {
       T4=0;
       max_steer_vel=sqrt(steer_vel*steer_vel/2.0+limits->max_steer_acc*(limits->max_steer_angle-steer_angle));
     }
   }
 }
 else
 {
  max_steer_angle=-steer_vel*steer_vel/(2.0*limits->max_steer_acc)+max_steer_vel*max_steer_vel/limits->max_steer_acc+steer_angle;
  T4=0;
  if(max_steer_angle>limits->max_steer_angle)
  {
   max_steer_angle=limits->max_steer_angle;
   max_steer_vel=sqrt(steer_vel*steer_vel/2.0+limits->max_steer_acc*(limits->max_steer_angle-steer_angle));
 }
}
min_steer_vel=(steer_vel-limits->max_steer_acc*sim_time_)/2.0;
if(min_steer_vel<=limits->min_steer_vel)
{
  min_steer_vel=limits->min_steer_vel;
  T4=sim_time_+(2.0*min_steer_vel-steer_vel)/limits->max_steer_acc;
  min_steer_angle=-min_steer_vel*min_steer_vel/limits->max_steer_acc+steer_vel*steer_vel/(2.0*limits->max_steer_acc)+min_steer_vel*T4+steer_angle;
  if(min_steer_angle<limits->min_steer_angle)
  {
   min_steer_angle=limits->min_steer_angle;
   T4=(limits->min_steer_angle-steer_angle+min_steer_vel*min_steer_vel/limits->max_steer_acc-steer_vel*steer_vel/(2.0*limits->max_steer_acc))/min_steer_vel;
   if(T4<0)
   {
     T4=0;
     min_steer_vel=-sqrt(steer_vel*steer_vel/2.0-limits->max_steer_acc*(limits->min_steer_angle-steer_angle));
   }
 }
}
else
{
  min_steer_angle=-min_steer_vel*min_steer_vel/limits->max_steer_acc+steer_vel*steer_vel/(2.0*limits->max_steer_acc)+steer_angle;
  T4=0;
  if(min_steer_angle<limits->min_steer_angle)
  {
   min_steer_angle=limits->min_steer_angle;
   min_steer_vel=-sqrt(steer_vel*steer_vel/2.0-limits->max_steer_acc*(limits->min_steer_angle-steer_angle)); 
 }
}
}
double Tacc,Tdeacc;
  // maximum translation speed
Tacc=(limits->max_trans_vel-trans_vel)/limits->max_trans_acc;
Tdeacc=limits->max_trans_vel/limits->max_trans_acc;
if((sim_time_-Tacc-Tdeacc)>0.0)
  max_trans_vel=limits->max_trans_vel;
else
  max_trans_vel=(sim_time_*limits->max_trans_acc+trans_vel)/2.0;
  // minimum translation speed
Tacc=-(limits->min_trans_vel-trans_vel)/limits->max_trans_acc;
Tdeacc=-(limits->min_trans_vel)/limits->max_trans_acc;
if((sim_time_-Tacc-Tdeacc)>0.0)
  min_trans_vel=limits->min_trans_vel;
else
  min_trans_vel=-(sim_time_*limits->max_trans_acc-trans_vel)/2.0;
  /* compute the margins */
Eigen::Vector2f sample = Eigen::Vector2f::Zero();
base_local_planner::VelocityIterator trans_vel_it(min_trans_vel, max_trans_vel, vsamples[0]);
base_local_planner::VelocityIterator steer_angle_it(min_steer_angle, max_steer_angle, vsamples[1]);
for(; !trans_vel_it.isFinished(); trans_vel_it++) 
{
  sample[0] = trans_vel_it.getVelocity();
  for(; !steer_angle_it.isFinished(); steer_angle_it++) 
  {
    sample[1] = steer_angle_it.getVelocity();
    sample_params_.push_back(sample);
  }
  steer_angle_it.reset();
}
}

void AckermannTrajectoryGenerator::set_parameters(
  double max_sim_time,
  double min_sim_time,
  double sim_granularity,
  double angular_sim_granularity,
  double sim_period) 
{
  max_sim_time_ = max_sim_time;
  min_sim_time_ = min_sim_time;
  sim_granularity_ = sim_granularity;
  angular_sim_granularity_ = angular_sim_granularity;
  sim_period_ = sim_period;
}

/**
 * Whether this generator can create more trajectories
 */
bool AckermannTrajectoryGenerator::hasMoreTrajectories() 
{
  return next_sample_index_ < sample_params_.size();
}

/**
 * Create and return the next sample trajectory
 */
bool AckermannTrajectoryGenerator::nextTrajectory(base_local_planner::Trajectory &comp_traj) 
{
  bool result = false;
  if (hasMoreTrajectories()) 
  {
    if (generate_trajectory(
      pos_,
      ackermann_,
      sample_params_[next_sample_index_],
      comp_traj)) 
    {
      result = true;
    }
  }
  next_sample_index_++;
  return result;
}

/**
 * @param pos current position of robot
 * @param vel desired velocity for sampling
 */
bool AckermannTrajectoryGenerator::generate_trajectory(
  Eigen::Vector3f pos,
  Eigen::Vector3f ackermann,
  Eigen::Vector2f sample_target_vel,
  base_local_planner::Trajectory& traj) 
{
  double x_i = pos[0];
  double y_i = pos[1];
  double theta_i = pos[2];
  // ackerman current state
  double trans_vel_i=ackermann[0];
  double steer_angle_i=ackermann[1],steer_vel_i=ackermann[2];

  traj.cost_   = -1.0; // placed here in case we return early
  //trajectory might be reused so we'll make sure to reset it
  traj.resetPoints();
  int num_steps = int(sim_time_ / sim_granularity_ + 0.5);
  //we at least want to take one step... even if we won't move, we want to score our current position
  if(num_steps == 0)
    num_steps = 1;
  /* compute trajectory times */
  double speed=0.0,angle=0.0;
  double T1=0.0,T4=0.0,T2=0.0,T3=0.0;

  /* check wether the trajectory can be generated or not */
/*  if(steer_vel_i<0 && (-steer_vel_i*steer_vel_i/(2*limits_->max_steer_acc)+steer_angle_i)>limits_->max_steer_angle)
  {
    ROS_WARN("Impossible steering trajectory: speed: %f, angle: %f",steer_vel_i,steer_angle_i);
    return false;
  }
  if(steer_vel_i>0 && (steer_vel_i*steer_vel_i/(2*limits_->max_steer_acc)+steer_angle_i)>limits_->max_steer_angle)
  {
    ROS_WARN("Impossible steering trajectory: speed: %f, angle: %f",steer_vel_i,steer_angle_i);
    return false;
  }
  if(steer_vel_i<0 && (-steer_vel_i*steer_vel_i/(2*limits_->max_steer_acc)+steer_angle_i)<limits_->min_steer_angle)
  {
    ROS_WARN("Impossible steering trajectory: speed: %f, angle: %f",steer_vel_i,steer_angle_i);
    return false;
  }
  if(steer_vel_i>0 && (steer_vel_i*steer_vel_i/(2*limits_->max_steer_acc)+steer_angle_i)<limits_->min_steer_angle)
  {
    ROS_WARN("Impossible steering trajectory: speed: %f, angle: %f",steer_vel_i,steer_angle_i);
    return false;
  }*/
  // compute the trajectory times
  if(steer_vel_i>=0)
  {
    if(sample_target_vel[1]>steer_angle_i)
    { 
      speed=(steer_vel_i+limits_->max_steer_acc*sim_time_)/2.0;
      if(speed>=limits_->max_steer_vel)
      {
        speed=limits_->max_steer_vel;  
        T4=sim_time_-(2.0*speed-steer_vel_i)/limits_->max_steer_acc;
        angle=speed*speed/limits_->max_steer_acc-steer_vel_i*steer_vel_i/(2.0*limits_->max_steer_acc)+speed*sim_time_*T4+steer_angle_i;
        if(angle>sample_target_vel[1])
        {
         angle=sample_target_vel[1];
         T4=(sample_target_vel[1]-steer_angle_i-speed*speed/limits_->max_steer_acc+steer_vel_i*steer_vel_i/(2.0*limits_->max_steer_acc))/speed;
         if(T4<0)
         {
           T4=0;
           speed=sqrt(steer_vel_i*steer_vel_i/2.0+limits_->max_steer_acc*(sample_target_vel[1]-steer_angle_i));
         }
       }
     }
     else
     { 
       angle=speed*speed/limits_->max_steer_acc-steer_vel_i*steer_vel_i/(2.0*limits_->max_steer_acc)+steer_angle_i;
       T4=0;
       if(angle>sample_target_vel[1])
       {
         angle=sample_target_vel[1];
         speed=sqrt(steer_vel_i*steer_vel_i/2.0+limits_->max_steer_acc*(sample_target_vel[1]-steer_angle_i));
       }
     }
     T1=fabs(speed-steer_vel_i)/limits_->max_steer_acc;
   }
   else
   {
    speed=(steer_vel_i-limits_->max_steer_acc*sim_time_)/2.0;
    if(speed<=limits_->min_steer_vel)
    {
     speed=limits_->min_steer_vel;
     T4=sim_time_+(2.0*speed-steer_vel_i)/limits_->max_steer_acc;
     angle=steer_vel_i*steer_vel_i/(2.0*limits_->max_steer_acc)-speed*speed/limits_->max_steer_acc+speed*T4+steer_angle_i;
     if(angle<sample_target_vel[1])
     {
       angle=sample_target_vel[1];
       T4=(sample_target_vel[1]-steer_angle_i+speed*speed/limits_->max_steer_acc-steer_vel_i*steer_vel_i/(2.0*limits_->max_steer_acc))/speed;
       if(T4<0)
       {
         T4=0;
         speed=-sqrt(steer_vel_i*steer_vel_i/2.0-limits_->max_steer_acc*(sample_target_vel[1]-steer_angle_i));
       }
     }
   }
   else
   {
     angle=steer_vel_i*steer_vel_i/(2.0*limits_->max_steer_acc)-speed*speed/limits_->max_steer_acc+steer_angle_i;
     T4=0;
     if(angle<sample_target_vel[1])
     {
       angle=sample_target_vel[1];
       speed=-sqrt(steer_vel_i*steer_vel_i/2.0-limits_->max_steer_acc*(sample_target_vel[1]-steer_angle_i)); 
     }
   }
   T1=fabs(speed-steer_vel_i)/limits_->max_steer_acc;
 }
}
else
{
  if(sample_target_vel[1]>steer_angle_i)
  {
    speed=(steer_vel_i+limits_->max_steer_acc*sim_time_)/2.0;
    if(speed>=limits_->max_steer_vel)
    {
     speed=limits_->max_steer_vel;
     T4=sim_time_-(2.0*speed-steer_vel_i)/limits_->max_steer_acc;
     angle=-steer_vel_i*steer_vel_i/(2.0*limits_->max_steer_acc)+speed*speed/limits_->max_steer_acc+speed*T4+steer_angle_i;
     if(angle>sample_target_vel[1])
     {
       angle=sample_target_vel[1];
       T4=(sample_target_vel[1]-steer_angle_i-speed*speed/limits_->max_steer_acc+steer_vel_i*steer_vel_i/(2.0*limits_->max_steer_acc))/speed;
       if(T4<0)
       {
         T4=0;
         speed=sqrt(steer_vel_i*steer_vel_i/2.0+limits_->max_steer_acc*(sample_target_vel[1]-steer_angle_i));
       }
     }
   }
   else
   {
     angle=-steer_vel_i*steer_vel_i/(2.0*limits_->max_steer_acc)+speed*speed/limits_->max_steer_acc+steer_angle_i;
     T4=0;
     if(angle>sample_target_vel[1])
     {
       angle=sample_target_vel[1];
       speed=sqrt(steer_vel_i*steer_vel_i/2.0+limits_->max_steer_acc*(sample_target_vel[1]-steer_angle_i));
     }
   }
   T1=fabs(speed-steer_vel_i)/limits_->max_steer_acc;
 }
 else
 {
  speed=(steer_vel_i-limits_->max_steer_acc*sim_time_)/2.0;
  if(speed<=limits_->min_steer_vel)
  {
   speed=limits_->min_steer_vel;
   T4=sim_time_+(2.0*speed-steer_vel_i)/limits_->max_steer_acc;
   angle=-speed*speed/limits_->max_steer_acc+steer_vel_i*steer_vel_i/(2.0*limits_->max_steer_acc)+speed*T4+steer_angle_i;
   if(angle<sample_target_vel[1])
   {
     angle=sample_target_vel[1];
     T4=(sample_target_vel[1]-steer_angle_i+speed*speed/limits_->max_steer_acc-steer_vel_i*steer_vel_i/(2.0*limits_->max_steer_acc))/speed;
     if(T4<0)
     {
       T4=0;
       speed=-sqrt(steer_vel_i*steer_vel_i/2.0-limits_->max_steer_acc*(sample_target_vel[1]-steer_angle_i));
     }
   }
 }
 else
 {
   angle=-speed*speed/limits_->max_steer_acc+steer_vel_i*steer_vel_i/(2.0*limits_->max_steer_acc)+steer_angle_i;
   T4=0;
   if(angle<sample_target_vel[1])
   {
     angle=sample_target_vel[1];
     speed=-sqrt(steer_vel_i*steer_vel_i/2.0-limits_->max_steer_acc*(sample_target_vel[1]-steer_angle_i)); 
   }
 }
 T1=fabs(speed-steer_vel_i)/limits_->max_steer_acc;
}
}

double v=0.0;
if(sample_target_vel[0]>trans_vel_i)
{
  v=(trans_vel_i+limits_->max_trans_acc*sim_time_)/2.0;
  if(v>sample_target_vel[0])
  {
    v=sample_target_vel[0];
    T3=sim_time_-(2.0*v-trans_vel_i)/limits_->max_trans_acc;
  }
  else
    T3=0;
  T2=(v-trans_vel_i)/limits_->max_trans_acc;
  if(T2<0)
  {
    ROS_WARN("Impossible drive trajectory: v_trans: %f",trans_vel_i);
    return false;
  }
}
else
{
  v=(trans_vel_i-limits_->max_trans_acc*sim_time_)/2.0;
  if(v<sample_target_vel[0])
  {
    v=sample_target_vel[0];
    T3=sim_time_+(2.0*v-trans_vel_i)/limits_->max_trans_acc;
  }
  else
    T3=0;
  T2=-(v-trans_vel_i)/limits_->max_trans_acc;
  if(T2<0)
  {
    ROS_WARN("Impossible drive trajectory: v_trans: %f",trans_vel_i);
    return false;
  }
}

  //compute a timestep
double dt = sim_time_ / num_steps;
double time=0.0;
traj.time_delta_ = dt;
traj.xv_     = sample_target_vel[0];
traj.yv_     = 0.0;
traj.thetav_ = sample_target_vel[1];

  //simulate the trajectory and check for collisions, updating costs along the way
for (int i = 0; i < num_steps; ++i) 
{
    //add the point to the trajectory so we can draw it later if we want
  traj.addPoint(x_i,y_i,theta_i);
    // compute the next point in the trajectory
  if(speed>ackermann[2])
  {
    if(time<T1)
    {
     steer_vel_i=steer_vel_i+limits_->max_steer_acc*dt;
     if(steer_vel_i>speed)
       steer_vel_i=speed;
   }
   else if(time<T1+T4)
     steer_vel_i=speed;
   else
   { 
     steer_vel_i=steer_vel_i-limits_->max_steer_acc*dt;
     if(steer_vel_i<0)
       steer_vel_i=0;
   }
 }
 else
 {
  if(time<T1)
  {
   steer_vel_i=steer_vel_i-limits_->max_steer_acc*dt;
   if(steer_vel_i<speed)
     steer_vel_i=speed;
 }
 else if(time<T1+T4)
   steer_vel_i=speed;
 else
 {
   steer_vel_i=steer_vel_i+limits_->max_steer_acc*dt;
   if(steer_vel_i>0)
     steer_vel_i=0;
 }
}
steer_angle_i+=steer_vel_i*dt;
if(v>ackermann[0])
{
  if(time<T2)
  {
   trans_vel_i=trans_vel_i+limits_->max_trans_acc*dt;
   if(trans_vel_i>v)
     trans_vel_i=v;
 }
 else if(time<(T2+T3))
   trans_vel_i=v;
 else
 {
   trans_vel_i=trans_vel_i-limits_->max_trans_acc*dt;
   if(trans_vel_i<0)
     trans_vel_i=0;
 }
}
else
{
  if(time<T2)
  {
   trans_vel_i=trans_vel_i-limits_->max_trans_acc*dt;
   if(trans_vel_i<v)
     trans_vel_i=v;
 }
 else if(time<T2+T3)
   trans_vel_i=v;
 else
 {  
   trans_vel_i=trans_vel_i+limits_->max_trans_acc*dt;
   if(trans_vel_i>0)
     trans_vel_i=0;
 }
}
double r,d;
if(fabs(steer_angle_i)>0.001)
{
  r=fabs(limits_->axis_distance/tan(steer_angle_i));
  d=trans_vel_i*dt;
  if(steer_angle_i>0)
    theta_i+=d/r;
  else
    theta_i-=d/r;
  x_i+=d*cos(theta_i);
  y_i+=d*sin(theta_i);
}
else
{
  d=trans_vel_i*dt;
  x_i+=d*cos(theta_i);
  y_i+=d*sin(theta_i);
}
time+=dt;
  } // end for simulation steps

  return num_steps > 0; // true if trajectory has at least one point
}

AckermannTrajectoryGenerator::~AckermannTrajectoryGenerator()
{

}
