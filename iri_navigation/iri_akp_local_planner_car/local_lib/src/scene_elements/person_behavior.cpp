/*
 * person_behavior.cpp
 *
 *  Created on: Dec 17, 2013
 *      Author: gferrer
 */
#include "scene_elements/person_behavior.h"
#include <math.h>
#include <iostream>

Cperson_behavior::Cperson_behavior(unsigned int id, Cperson_abstract::target_type person_target_type,
		Cperson_abstract::force_type person_force_type, double _time_window) :
    Cperson_bhmip(id,person_target_type, person_force_type, _time_window)
{
	prediction_trajectory_.reserve(100);//default value for the prediction trajectory
}

Cperson_behavior::~Cperson_behavior()
{

}

void Cperson_behavior::add_pointV( SpointV_cov point ,Cperson_abstract::filtering_method filter)
{
	Cperson_bhmip::add_pointV(point,filter);
	//calculate expected behavior is done in the estimation methd (in scene)
}

void Cperson_behavior::prediction(double min_v_to_predict)
{
	Cperson_bhmip::prediction( min_v_to_predict);
	//behavior estimation is required to be done jointly with all person, so it is calculated outside the class
}

Sbehavior* Cperson_behavior::find_behavior_estimation( unsigned int id )
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
Cperson_behavior::get_best_behavior_to_person( unsigned int interacting_person ) const
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

	return result;
}

void Cperson_behavior::prediction_propagation( double dt , Sforce force , unsigned int index )
{
	prediction_trajectory_.push_back( prediction_trajectory_.at(index).propagate(dt,force,desired_velocity_) );
}

void Cperson_behavior::planning_propagation( double dt , Sforce force , unsigned int index )
{
	planning_trajectory_.push_back( planning_trajectory_.at(index).propagate(dt,force,desired_velocity_) );
}

void Cperson_behavior::planning_propagation_copy( unsigned int prediction_index )
{
	planning_trajectory_.push_back( prediction_trajectory_.at(prediction_index) );
}

bool Cperson_behavior::is_needed_to_propagate_person_for_planning( unsigned int parent_index, Spoint robot, unsigned int& new_index_to_be_copied )
{
	bool res;
	//TODO first iteration: only checks distance to target
	if ( robot.distance( planning_trajectory_.at( parent_index ) ) < 1.0 && !has_copied_propagation_ )
	{
		res = true;
	}
	else
	{
		has_copied_propagation_ = true;
		unsigned int index(0);
		while( robot.time_stamp > prediction_trajectory_.at(index).time_stamp  && index < prediction_trajectory_.size()-1 )
		{
			index++;
		}
		new_index_to_be_copied = index;
		res = false;
	}

	return res;
}

void Cperson_behavior::clear_prediction_trajectory()
{
	prediction_trajectory_.clear();
	prediction_trajectory_.push_back( current_pointV_ );
	planning_trajectory_.clear();
	planning_trajectory_.push_back( current_pointV_ );
	has_copied_propagation_ = false;
}

void Cperson_behavior::clear_planning_trajectory()
{
	planning_trajectory_.clear();
	planning_trajectory_.push_back( current_pointV_ );
	has_copied_propagation_ = false;
}

void Cperson_behavior::reserve_prediction_trajectory( unsigned int n)
{
	prediction_trajectory_.reserve( n );//if n < capacity() then does nothing
}

void Cperson_behavior::reset(  )
{
	Cperson_bhmip::reset();
	expected_behavior_list_.clear();
	prediction_trajectory_.clear();
	planning_trajectory_.clear();
}


