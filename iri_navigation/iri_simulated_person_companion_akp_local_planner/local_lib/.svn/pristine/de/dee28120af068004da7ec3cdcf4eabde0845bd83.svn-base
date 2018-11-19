/*
 * force_reactive_robot_companion.cpp
 *
 *  Created on: Nov 9, 2013
 *      Author: gonzalo
 */


#include "nav/force_reactive_robot_companion.h"
#include <math.h>


Cforce_reactive_robot_companion::Cforce_reactive_robot_companion ( ) :
	Cprediction_bhmip(),
	target_person_id_(0),
	robot_state_( UNKNOWN_ZONE ),
	v_max_(2.0),
	v_cruise_(1.0),
	time_horizon_(0.2),
	is_target_person_visible_(false)
{
	param_force_approach_.push_back(0.1);//force goal
	param_force_approach_.push_back(1.0);//force persongoal
	param_force_approach_.push_back(7.0);//force interaction persons
	param_force_approach_.push_back(1.0);//force interaction laser
	param_force_approach_.push_back(1.0);//force interaction map


}

Cforce_reactive_robot_companion::~Cforce_reactive_robot_companion()
{
	//parent class do their destructors and no need to do anything free memory in this class
}

SpointV_cov Cforce_reactive_robot_companion::robot_approach_pose(bool stop_if_smn_in_inner)
{
	Sforce f1,f2,f3,f4,f5,f;
	return robot_approach_pose(f1,f2,f3,f4,f5,f,stop_if_smn_in_inner );
}

SpointV_cov Cforce_reactive_robot_companion::robot_approach_pose( Sforce& f_goal, Sforce& f_persongoal,
	Sforce& f_int_pers, Sforce& f_int_laser, Sforce& f_int_map, Sforce& f , bool stop_if_smn_in_inner)
{
	//get initial conditions
	SpointV_cov robot = robot_.get_current_pointV();
	SpointV_cov approach;
	approach.x = robot.x;
	//approach.x = 1.0/0.0; //sending invalid commands to stop movement
	approach.y = robot.y;
	//approach.y = 1.0/0.0;
	approach.time_stamp = robot.time_stamp;
	approach.vx = 0.0; //no approach, stays at current position at v=0 TODO
	approach.vy = 0.0;
	Cperson_abstract* person_obj;

	//checking if stop_if_smn_in_inner flag is set
	if( stop_if_smn_in_inner )
	{
		Cperson_abstract* nearest = nearest_person( robot_ );
		double phi = robot_->get_current_pointV().angle_heading_point( nearest->get_current_pointV() );
		double lambda = 0.5;
		double anisotropy = (lambda + (1-lambda)*(1 + cos(phi))/2 );
		if( distance_to_nearest_person(robot_) * anisotropy < 0.8  )
		{
			return approach;
		}
	}

	if (!find_person(target_person_id_ , &person_obj)  )
	{
		is_target_person_visible_ = false;
		robot_state_ = UNKNOWN_ZONE;
		f_goal = Sforce();
		f_persongoal = Sforce();
		f_int_pers = force_persons_int( robot_  ) * param_force_approach_[2];
		f_int_laser = force_objects_laser_int( robot_ ) * param_force_approach_[3];
		f_int_map =  get_force_map_robot_position()* param_force_approach_[4];
		f = f_int_pers + f_int_map + f_int_laser;
		return approach;
	}
	is_target_person_visible_ = true;
	SpointV_cov person = person_obj->get_current_pointV();

	//calculate current robot destinations:
	std::vector<Sdestination> dest;
	dest.push_back( person_obj->get_best_dest() );
	//calculation of the person destination
	person_destination_ = Sdestination(0, person.x ,person.y,1.0);
	//positive difference means on the left hand side of the angle circle thus, positive turn
	//if( diffangle(person.theta, atan2(robot.y-person.y , robot.x-person.x) ) < 0 )

	//a pure geometric approach: makes the destination more stable in time
	double theta = atan2(person_obj->get_best_dest().y-person.y ,
						person_obj->get_best_dest().x-person.x);
	if( diffangle(theta, atan2(robot.y-person.y , robot.x-person.x) ) < 0 )
	{
		//person_destination_.x += r_*cos(person.theta + ro_);
		person_destination_.x += r_*cos(theta + ro_);
		person_destination_.y += r_*sin(theta + ro_);
	}
	else
	{
		person_destination_.x += r_*cos(theta - ro_);
		person_destination_.y += r_*sin(theta - ro_);
	}
	dest.push_back( person_destination_ );
	robot_->set_destinations( dest );

	//calculate robot state and if unknown, do not move
	set_robot_state(person,robot,  get_sfm_params(robot_)->at(1) );

	if ( robot_state_ == UNKNOWN_ZONE)
		return approach;

	//calculate robot forces
	if (robot.v() < 0.2 ){ //just to avoid
		robot.norm_v( v_cruise_);
	}
	//const std::vector<double>* social_forces_param = person_obj->get_social_force_parameters_person_robot();
	robot_->set_desired_velocty(v_robot_desired_);
	f_goal = robot_->force_goal( person_obj->get_best_dest(), get_sfm_params(&robot_) )* param_force_approach_[0];
	f_persongoal = robot_->force_goal( person_destination_ , get_sfm_params(&robot_) )* param_force_approach_[1];
	//f_persongoal = robot_force_persongoal(robot, person, social_forces_param)* param_force_approach_[1];
	f_int_pers = force_persons_int( &robot_  ) * param_force_approach_[2];
	f_int_laser =   force_objects_laser_int( &robot_ )*param_force_approach_[3];
	f_int_map =   get_force_map_robot_position()* param_force_approach_[4];
	double w = 1.0;
	do
	{
		f_goal = f_goal * w;
		f_persongoal = f_persongoal * w;
		f_int_pers = f_int_pers *w;
		f = f_goal + f_persongoal +	f_int_pers + f_int_map + f_int_laser;
		approach = robot_tangential_propagation( f );
		w *= 0.9;
	} while( read_force_map_success_ && !is_cell_clear_map( approach.x, approach.y ) && w > 0.1 );
	//approach = get_robot().pose_propagation( dt_, f );
	return approach;
}

bool Cforce_reactive_robot_companion::set_nearest_target_person()
{
	//returns True if the target was set and false if no person was in the scene nearer than 20m
	Cperson_abstract* nearest = nearest_person( &robot_ );
	if ( nearest->get_current_pointV().distance( robot_.get_current_pointV() )  < 20.0 && *nearest != robot_ )
	{
		target_person_id_ = nearest->get_id();
		return true;
	}
	else
	{
		return false;
	}
}

Cperson_abstract* Cforce_reactive_robot_companion::nearest_person( Cperson_abstract* person)
{
	Cperson_abstract* nearest_person = person;
	double d = 2000, d_to_person;
	for( std::list<Cperson_abstract*>::iterator iit = person_list_.begin() ; iit != person_list_.end() ; iit++ )
	{
		d_to_person = (*iit)->get_current_pointV().distance( person->get_current_pointV() );
		if ( d_to_person < d && person != *iit )
		{
			d = d_to_person;
			nearest_person = *iit;
		}
	}
	return nearest_person;
}

double Cforce_reactive_robot_companion::distance_to_nearest_person( Cperson_abstract* person )
{
	Cperson_abstract* nearest = nearest_person( person );
	if (nearest == person)
		return -1.0;
	return nearest->get_current_pointV().distance( person->get_current_pointV() );

}


Sforce Cforce_reactive_robot_companion::robot_force_persongoal(SpointV_cov robot, SpointV_cov person, std::vector<double> social_forces_param )
{
	//force calculation
	robot_.set_desired_velocty( v_robot_desired_ );
	return robot_.force_goal(  person_destination_, get_sfm_params(&robot_) );
}

void Cforce_reactive_robot_companion::set_robot_state( SpointV_cov person, SpointV_cov robot , double lambda )
{
	//calculation of the influence zone
	double phi = diffangle( person.orientation() , atan2(robot.y-person.y , robot.x-person.x) );
	double anisotropy = (lambda + (1-lambda)*(1 + cos(phi))/2 );

	if ( anisotropy*1.5 > robot.distance(person)  )
		robot_state_ = INNER_ZONE;
	else if ( anisotropy*3 > robot.distance(person) )
		robot_state_ = MID_ZONE;
	else
		robot_state_ = OUT_ZONE;

	switch(robot_state_)
	{
		case INNER_ZONE:
			//v_robot_desired_ = 1.5*person.v;
			v_robot_desired_ = v_cruise_;
			break;
		case MID_ZONE:
			//v_robot_desired_ = 2 * person.v;
			v_robot_desired_ = 1.5*v_cruise_;
			if (v_robot_desired_ < 0.2)//almost stopped
				v_robot_desired_ = v_cruise_;
			break;
		case OUT_ZONE:
			v_robot_desired_ = v_max_;
			break;
		case UNKNOWN_ZONE:
		default:
			v_robot_desired_ = 0.0;
			break;
	}
	robot_.set_desired_velocty( v_robot_desired_ );

}

SpointV_cov Cforce_reactive_robot_companion::robot_tangential_propagation(Sforce f)
{
	//returns the propagated state given a time horizon of propagation
	// and assuming no other force is applied, except for the initial one
	SpointV_cov robot = robot_.get_current_pointV();

	//limits max acceleration
	//if (f.module() > a_max_)
	//	f = f * (a_max_ / f.module());

	//limits max velocity
	double vx = robot.vx + f.fx * dt_;
	double vy = robot.vy + f.fy * dt_;
	double v = robot.v();
	if (v > v_max_)
	{
		vx *= v_max_/v;
		vy *= v_max_/v;
		v = v_max_;
	}
	//check for destination proximity
	double time_propagated;
	Sdestination dest = robot_.get_destinations()->at(1);//person destination
	double distance = robot.distance( SpointV_cov(dest.x,dest.y) );

	if (distance / v < time_horizon_) //propagated pose would be farther than destination
	{
		//time_propagated = 0.1; //NO FUNCIONA!robot remains in place, to avoid minor position changes that entail huge orientation differences...
		if (v < 0.05 ) v = 0.05;
		time_propagated = distance / v;
	}
	else
	{
		time_propagated = time_horizon_;
	}
	return SpointV_cov(robot.x + vx * time_propagated , robot.y + vy * time_propagated,
                                                                robot.time_stamp , vx, vy );

}
void Cforce_reactive_robot_companion::set_force_params( double force_goal, double force_toperson, double force_interaction)
{
	param_force_approach_[0] = force_goal;
	param_force_approach_[1] = force_toperson;
	param_force_approach_[2] = force_interaction;
}

void Cforce_reactive_robot_companion::set_force_params( double force_goal, double force_toperson)
{
	param_force_approach_[0] = force_goal;
	param_force_approach_[1] = force_toperson;
}

void Cforce_reactive_robot_companion::set_force_params( double force_goal, double force_toperson,
			double force_interaction, double force_laser, double force_map)
{
	param_force_approach_[0] = force_goal;
	param_force_approach_[1] = force_toperson;
	param_force_approach_[2] = force_interaction;
	param_force_approach_[3] = force_laser;
	param_force_approach_[4] = force_map;
}
void Cforce_reactive_robot_companion::robot_parameters_feedback_adjustment( double feedback_sign )
{
	//feedback_sing input is a double float of value 1.0 or -1.0

	switch( robot_state_ )
	{
		case INNER_ZONE:
			param_force_approach_[2] -= feedback_sign * 2.0;
			if( param_force_approach_[2] < 0 )
				param_force_approach_[2] = 0.0;
			break;

		case MID_ZONE:
		case OUT_ZONE:
		case UNKNOWN_ZONE:
			param_force_approach_[0] -=  feedback_sign * 0.1;
			if (param_force_approach_[0] > 1.0)
				param_force_approach_[0] = 1.0;
			if (param_force_approach_[0] < 0.0)
				param_force_approach_[0] = 0.0;

			param_force_approach_[1] += feedback_sign * 0.1;
			if (param_force_approach_[1] > 1.0)
				param_force_approach_[1] = 1.0;
			if (param_force_approach_[1] < 0.0)
				param_force_approach_[1] = 0.0;

			/*v_max_ += feedback_sign * 0.14;
			if (v_max_ > 1.6)
				v_max_ = 1.6;
			if (v_max_ < 0.6)
				v_max_ = 0.6;*/
			break;

		default:
			//do nothing
			break;
	}
}


