/*
 * robot.cpp
 *
 *  Created on: Jul 31, 2013
 *      Author: gferrer
 */
#include "scene_elements/robot.h"
#include <math.h>

Crobot::Crobot(  unsigned int id , robot_type robot_type,
		Cperson_abstract::force_type person_force_type, double _time_window) :
	Cperson_abstract( id, Cperson_abstract::Robot, person_force_type),
	v_max_(1.0), w_max_(3.0), a_v_max_(2.0), a_v_break_(2.0),a_w_max_(5.0),
	platform_radii_(1.0), platform_radii_2_(1.0)
{
}

Crobot::~Crobot()
{

}

void Crobot::add_pose( Spose  observation )
{
	//TODO take into account covariances
	diff_pose_ = observation - current_pose_;
	current_pose_ = observation;
	SpointV_cov current_point = SpointV_cov( observation.x, observation.y,observation.time_stamp,
			observation.v*cos(observation.theta), observation.v*sin(observation.theta) );
	diff_pointV_ = current_point - current_pointV_ ;
	current_pointV_ = current_point;
}

void Crobot::prediction( double min_v_to_predict)
{

}

void Crobot::reset()
{
	current_pointV_ = SpointV_cov();
	current_pose_ = Spose();
	diff_pointV_ = SpointV_cov();
	best_destination_ = Sdestination();
	destinations_.clear();
	planning_trajectory_.clear();
	planning_SpointV_trajectory_.clear();
	rnd_goal_ = Spoint();
}

void Crobot::clear_planning_trajectory()
{
    planning_trajectory_.clear();
    planning_trajectory_.push_back(current_pose_);
    planning_SpointV_trajectory_.clear();
    planning_SpointV_trajectory_.push_back( current_pointV_);


}

void Crobot::erase_last_planning_propagation()
{
    planning_trajectory_.pop_back();
    planning_SpointV_trajectory_.pop_back();
}

void Crobot::robot_propagation(double dt , unsigned int index, double v, double w)
{
	//Kinematic unicycle TODO propagate covariances
	Spose propagated_pose,ini_pose;
	ini_pose = planning_trajectory_.at(index);
	if( v > v_max_) v=v_max_;
	if( v < -v_max_) v=-v_max_;
	if( w > w_max_) w=w_max_;
	if( w < -w_max_) w=-w_max_;
	propagated_pose.x = ini_pose.x + v*cos(ini_pose.theta)*dt;
	propagated_pose.y = ini_pose.y + v*sin(ini_pose.theta)*dt;
	propagated_pose.theta = ini_pose.theta + w*dt;
	propagated_pose.time_stamp = ini_pose.time_stamp + dt;
	propagated_pose.v = v;
	propagated_pose.w = w;
	//update
	planning_trajectory_.push_back(propagated_pose);
	planning_SpointV_trajectory_.push_back( SpointV_cov(propagated_pose.x,propagated_pose.y,
				propagated_pose.time_stamp, propagated_pose.v*cos(propagated_pose.theta),
				propagated_pose.v*sin(propagated_pose.theta)) );


}

void Crobot::robot_propagation(double dt , unsigned int index, const Sforce &f )
{
	//kinodynamic unicycle
	Spose propagated_pose,ini_pose;
	ini_pose = planning_trajectory_.at(index);

	//convert 2D forces into robot forces with non-holonomic contraints
	double av = cos(ini_pose.theta)*f.fx + sin(ini_pose.theta)*f.fy;//projection to robot pose (dot product)
	double aw = -sin(ini_pose.theta)*f.fx + cos(ini_pose.theta)*f.fy;//cross product theta x f

	// calculate the proportional torque thau = k (0 - w) similar to the steering force
	aw -= 1.6*ini_pose.w;//TODO this dampening parameter depends on the velocity v, w! careful if it changes significantly


	// check for forces validity
	if( av*ini_pose.v < 0.001 )//breaking configuration if different sign v and a
	{
		if( av > a_v_break_) av=a_v_break_;
		if( av < -a_v_break_) av=-a_v_break_;
	}
	else // not breaking or 0.0
	{
		if( av > a_v_max_) av=a_v_max_;
		if( av < -a_v_max_) av=-a_v_max_;
	}
	if( aw > a_w_max_) aw=a_w_max_;
	if( aw < -a_w_max_) aw=-a_w_max_;

	// update velocities
	propagated_pose.v = ini_pose.v + av*dt;
	propagated_pose.w = ini_pose.w + aw*dt;
	if( propagated_pose.v > v_max_) propagated_pose.v=v_max_;
	if( propagated_pose.v < -v_max_) propagated_pose.v=-v_max_;
	if( propagated_pose.w > w_max_) propagated_pose.w=w_max_;
	if( propagated_pose.w < -w_max_) propagated_pose.w=-w_max_;
	propagated_pose.x = ini_pose.x + ini_pose.v*cos(ini_pose.theta)*dt
			+ dt*dt/2.0*cos(ini_pose.theta)*av;
	propagated_pose.y = ini_pose.y + ini_pose.v*sin(ini_pose.theta)*dt
			+ dt*dt/2.0*sin(ini_pose.theta)*av;
	propagated_pose.theta = ini_pose.theta + ini_pose.w*dt
			+ dt*dt/2.0*aw;;
	propagated_pose.time_stamp = ini_pose.time_stamp + dt;

	//update
	planning_trajectory_.push_back(propagated_pose);
	planning_SpointV_trajectory_.push_back( SpointV_cov(propagated_pose.x,propagated_pose.y,
				propagated_pose.time_stamp, propagated_pose.v*cos(propagated_pose.theta),
				propagated_pose.v*sin(propagated_pose.theta)) );
}

Sbehavior* Crobot::find_behavior_estimation( unsigned int id )
{
	std::list<Sbehavior>::iterator iit = expected_behavior_list_.begin();
	Sbehavior *behavior;
	if ( !expected_behavior_list_.empty() )
	{
		for( ; iit != expected_behavior_list_.end(); iit++ )
		{
			if ( iit->related_person_id == id )
			{
				behavior = &(*iit);
				assert( behavior != NULL );
				return  behavior;
			}
			if ( iit->related_person_id > id )
				break;
		}
	}
	//behavior not found for person id, then a new behavior is set
	expected_behavior_list_.insert( iit, Sbehavior( id ) );
	iit--;//element inserted is before iit, so we want the pointer to the inserted element
	behavior = &(*iit);
	assert( behavior != NULL );
	return  behavior;
}

Cperson_abstract::behavior_type
Crobot::get_best_behavior_to_person( unsigned int interacting_person ) const
{
	Cperson_abstract::behavior_type result = Cperson_abstract::Balanced;
	const Sbehavior * behavior = NULL;
	std::list<Sbehavior>::const_iterator iit = expected_behavior_list_.begin();
	if ( !expected_behavior_list_.empty() )
	{
		for( ; iit != expected_behavior_list_.end(); iit++ )
		{
			if ( iit->related_person_id == interacting_person )
			{
				behavior = &(*iit);
				assert( behavior != NULL );
			}
			if ( iit->related_person_id > interacting_person )
				break;//person not found, so result is a balanced behavior
		}
	}
	if ( behavior == NULL ) return result;
	assert( behavior != NULL );
	double best_expectation =  behavior->expectation[0];
	for ( unsigned int i = 1; i < behavior->expectation.size(); ++i  )
	{

		if( best_expectation < behavior->expectation[i]  )
		{
			best_expectation = behavior->expectation[i];
			result = (Cperson_abstract::behavior_type)i;
		}
	}
	//behavior->print();
	return result;
}


void Crobot::correct_state_to_delay( Spose last_control_cmd, double now, double delay )
{
	current_pose_.w = last_control_cmd.w + (current_pose_.w - last_control_cmd.w ) * exp( - fabs(now - current_pose_.time_stamp + delay) / 0.05 );
	current_pose_.v = last_control_cmd.v + (current_pose_.v - last_control_cmd.v ) * exp( - fabs(now - current_pose_.time_stamp + delay) / 0.05 );
	//current_pointV_ not necessary for dynamic constraints, only for comparisions
	//TODO propagate state {x,y,theta}, not implemented yet
}

