/*
 * person_virtual.cpp
 *
 *  Created on: Jul 9, 2013
 *      Author: gferrer
 */
#include "scene_elements/person_virtual.h"

Cperson_virtual::Cperson_virtual( unsigned int id, Cperson_abstract::target_type person_target_type,
		Cperson_abstract::force_type person_force_type,
		double _time_window ) :
	Cperson_abstract(id,person_target_type, person_force_type)
{

}

Cperson_virtual::~Cperson_virtual()
{

}


void Cperson_virtual::add_pointV( SpointV_cov point, Cperson_abstract::filtering_method filter )
{
	diff_pointV_ = point - current_pointV_ ;
	//resets covariance to predefined values, if not,
	//it is a growing covariance function due to feedback propagation
	current_pointV_ = SpointV_cov();
	current_pointV_.x = point.x;
	current_pointV_.y = point.y;
	current_pointV_.vx = point.vx;
	current_pointV_.vy = point.vy;
	current_pointV_.time_stamp = point.time_stamp;
	now_ = current_pointV_.time_stamp;
}

void Cperson_virtual::prediction( double min_v_to_predict)
{

}

void Cperson_virtual::reset()
{
	current_pointV_ = SpointV_cov();
	diff_pointV_ = SpointV_cov();
	best_destination_ = Sdestination();
	destinations_.clear();
}
