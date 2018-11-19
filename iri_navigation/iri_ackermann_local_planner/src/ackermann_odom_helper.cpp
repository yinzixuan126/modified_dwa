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
#include <ackermann_odom_helper.h>

AckermannOdomHelper::AckermannOdomHelper(std::string odom_topic,int num_avg_samples) 
{
  set_odom_topic( odom_topic );
  last_states.resize(num_avg_samples);
}

void AckermannOdomHelper::odom_callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
  static int current_index=0;
  ROS_INFO_ONCE("odom received!");

  //we assume that the odometry is published in the frame of the base
  boost::mutex::scoped_lock lock(odom_mutex_);
  this->last_states[current_index].trans_speed=msg->twist.twist.linear.z;
  this->last_states[current_index].steer_angle=msg->twist.twist.angular.x;
  this->last_states[current_index].steer_speed=msg->twist.twist.angular.y;
  current_index=(current_index+1)%this->last_states.size();
  base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
  base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
  base_odom_.twist.twist.linear.z = msg->twist.twist.linear.z;// ackermann forward speed
  base_odom_.twist.twist.angular.x = msg->twist.twist.angular.x;//ackermann steer angle
  base_odom_.twist.twist.angular.y = msg->twist.twist.angular.y;//ackermann steer speed
  base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
}

//copy over the odometry information
void AckermannOdomHelper::get_odom(nav_msgs::Odometry& base_odom) {
  boost::mutex::scoped_lock lock(odom_mutex_);
  base_odom = base_odom_;
}

void AckermannOdomHelper::get_robot_vel(tf::Stamped<tf::Pose>& robot_vel) {
  // Set current velocities from odometry
  geometry_msgs::Twist global_vel;
  {
    boost::mutex::scoped_lock lock(odom_mutex_);
    global_vel.linear.x = base_odom_.twist.twist.linear.x;
    global_vel.linear.y = base_odom_.twist.twist.linear.y;
    global_vel.angular.z = base_odom_.twist.twist.angular.z;

    robot_vel.frame_id_ = base_odom_.child_frame_id;
  }
  robot_vel.setData(tf::Transform(tf::createQuaternionFromYaw(global_vel.angular.z), tf::Vector3(global_vel.linear.x, global_vel.linear.y, 0)));
  robot_vel.stamp_ = ros::Time();
}

void AckermannOdomHelper::get_ackermann_state(TAckermannState &state)
{
  unsigned int i=0;

  boost::mutex::scoped_lock lock(odom_mutex_);
  state.trans_speed=0;
  state.steer_angle=0;
  state.steer_speed=0;
  for(i=0;i<this->last_states.size();i++)
  {
    state.trans_speed+=this->last_states[i].trans_speed;
    state.steer_angle+=this->last_states[i].steer_angle;
    state.steer_speed+=this->last_states[i].steer_speed;
  }
  state.trans_speed/=this->last_states.size();
  state.steer_angle/=this->last_states.size();
  state.steer_speed/=this->last_states.size();
}

void AckermannOdomHelper::set_odom_topic(std::string odom_topic)
{
  if( odom_topic != odom_topic_ )
  {
    odom_topic_ = odom_topic;

    if( odom_topic_ != "" )
    {
      ros::NodeHandle gn;
      odom_sub_ = gn.subscribe<nav_msgs::Odometry>( odom_topic_, 1, boost::bind( &AckermannOdomHelper::odom_callback, this, _1 ));
    }
    else
    {
      odom_sub_.shutdown();
    }
  }
}

void AckermannOdomHelper::set_average_samples(int num_average_samples)
{
  boost::mutex::scoped_lock lock(odom_mutex_);
  if(num_average_samples!=this->last_states.size())
    this->last_states.resize(num_average_samples);
}

std::string AckermannOdomHelper::get_odom_topic(void) const
{
  return this->odom_topic_;
}

int AckermannOdomHelper::get_average_samples(void)
{
  return this->last_states.size();
}

AckermannOdomHelper::~AckermannOdomHelper()
{

}
