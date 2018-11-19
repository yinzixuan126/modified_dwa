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

#include <heading_cost_function.h>
#include <math.h>
#include <limits.h>
#include <values.h>

HeadingCostFunction::HeadingCostFunction() 
{
  this->num_points=1;
}

bool HeadingCostFunction::prepare() 
{
  return true;
}

double HeadingCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj) 
{
  double dist,x,y,theta,near_dist=DBL_MAX,heading_diff=0.0,diff=0.0;
  int interval=traj.getPointsSize()/this->num_points;
  unsigned int near_index=0,i,j;

  for(j=0;j<this->num_points;j++)
  {
    traj.getPoint((j+1)*interval-1,x,y,theta);
    // find the nearrest point on the path
    for(i=1;i<this->global_plan.size();i++)
    {
      dist=sqrt((this->global_plan[i].pose.position.x-x)*(this->global_plan[i].pose.position.x-x)+
                (this->global_plan[i].pose.position.y-y)*(this->global_plan[i].pose.position.y-y));
      if(dist<near_dist)
      {
	near_dist=dist;
	near_index=i;
      }
    }
    double v1_x,v1_y;
    v1_x = this->global_plan[near_index].pose.position.x - this->global_plan[near_index-1].pose.position.x;
    v1_y = this->global_plan[near_index].pose.position.y - this->global_plan[near_index-1].pose.position.y;
    double v2_x = cos(theta);
    double v2_y = sin(theta);

    double perp_dot = v1_x * v2_y - v1_y * v2_x;
    double dot = v1_x * v2_x + v1_y * v2_y;

    //get the signed angle
    diff = fabs(atan2(perp_dot, dot));
    if(diff>(3.14159/2.0))
      diff=fabs(diff-3.14159);
    heading_diff+=diff;
  }

  return heading_diff;
}

void HeadingCostFunction::set_global_plan(std::vector<geometry_msgs::PoseStamped> &plan)
{
  this->global_plan=plan;
}

void HeadingCostFunction::set_num_points(int num_points)
{
  if(num_points>0)
    this->num_points=num_points;
}

HeadingCostFunction::~HeadingCostFunction() 
{

}

