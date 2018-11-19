#include "prediction_bhmip.h"
#include "scene_elements/person_bhmip.h"
#include <math.h>
#include <algorithm>
#include <stdio.h>
#include <Eigen/Dense>
#include <iostream>



Cprediction_bhmip::Cprediction_bhmip( Cperson_abstract::filtering_method filter,
		Cperson_abstract::update_person_method update_method ) :
		Cscene_abstract(filter, update_method, Cperson_abstract::Spherical)
{
	//read destinations from txt
	//this->set_destinations(dest);
}

Cprediction_bhmip::~Cprediction_bhmip()
{
	for( std::list<Cperson_abstract*>::iterator iit = person_list_.begin() ; iit != person_list_.end() ; iit++ )
	{
		delete (*iit);
	}
	delete robot_;
}


void Cprediction_bhmip::update_scene(const std::vector<SdetectionObservation>& observation )
{
	// the filter bool sets if we want to filter velocities or use the provided in the update.
	//associate current targets with tracking observations
	Cperson_abstract* found_person;
	SdetectionObservation  point;
	for(unsigned int i=0 ; i< observation.size() ; ++i)
	{
		//std::cout << "iteration" << i << std::endl;
		if( find_person( observation[i].id , &found_person ) )
		{
			//std::cout << "updating target after observation " << found_person->is_observation_updated() << std::endl;
			point = observation[i];
			found_person->add_pointV( point, filtering_method_);
			now_ = observation[i].time_stamp;
		}
		//new targets
		else
		{
			found_person = add_person( observation[i].id );
			//std::cout << "new person :" << found_person->get_id() << std::endl;
			point = observation[i];
			found_person->add_pointV( point , filtering_method_);
			now_ = observation[i].time_stamp;
		}
	}

	//Remove persons
	std::vector<unsigned int> removed_targets;
	for(std::list<Cperson_abstract*>::iterator iit = person_list_.begin() ; iit!=person_list_.end() ; iit++)
	{
		//std::cout << "analyzing target " << (*iit)->is_observation_updated() << std::endl;
		if ( (*iit)->is_observation_updated() )
		{
			(*iit)->set_observation_update( false );//resets the observation flag until next update
			//std::cout << " updating target " << (*iit)->is_observation_updated() << std::endl;
		}
		else
		{
			removed_targets.push_back((*iit)->get_id());
			//std::cout << "trying to remove target " << (*iit)->get_id() << std::endl;
		}
	}
	for ( unsigned int i = 0; i < removed_targets.size() ; ++i)
	{
		if( update_method_ == Cperson_abstract::Autoremove )
		{
			remove_person(removed_targets[i]);
		}
		else
		{
			if( find_person(removed_targets[i], &found_person ) )
				found_person->refresh_person(now_);
		}
	}
	//else remove MUST be set manually outside the update function, each time the user wants to remove a person


}

void Cprediction_bhmip::rotate_and_traslate_scene(unsigned int id, double R, double thetaZ, double linear_vx, double linear_vy, double v_rot_x, double v_rot_y, std::vector<double> vect_odom_eigen_tf, bool debug)
{ 	// Function for local tracking. This function change the frame of the Current_pose and the window of poses for a concrete person (id), tacking into account the actual position of the robot.
	//  id ->person id ; R -> Robot Translation ; thetaZ -> Robot rotation

	Cperson_abstract* found_person;
	if( find_person( id , &found_person ) )
	{
		found_person->rotate_and_translate_trajectory(R,thetaZ,linear_vx,linear_vy,v_rot_x,v_rot_y, vect_odom_eigen_tf,debug);
	}
}

Cperson_abstract* Cprediction_bhmip::add_person_container( unsigned int id,
		std::list<Cperson_abstract*>::iterator iit )
{
	Cperson_bhmip* person = new Cperson_bhmip(id,Cperson_abstract::Person, scene_force_type_, filtering_time_window_);

	person_list_.insert( iit , person );
	person->set_destinations(destinations_);

	return (Cperson_abstract*) person;
}


void Cprediction_bhmip::scene_intentionality_prediction_bhmip()
{
	for( std::list<Cperson_abstract*>::iterator iit = person_list_.begin() ; iit != person_list_.end() ;  iit++ )
	{
		(*iit)->prediction( min_v_to_predict_);
	}

}


void Cprediction_bhmip::clear_scene()
{

}
