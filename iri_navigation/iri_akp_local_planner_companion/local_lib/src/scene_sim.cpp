/*
 * scene_sim.cpp
 *
 *  Created on: Mar 27, 2013
 *      Author: gferrer
 */
#include "scene_sim.h"
#include "scene_elements/person_virtual.h"
#include "random/rand_gmm.h"
//#include <iostream>

Cscene_sim::Cscene_sim(  ): Cscene_abstract(Cperson_abstract::No_filtering, Cperson_abstract::Autoremove,
		Cperson_abstract::Spherical),
  number_virtual_people_(0) , virtual_person_id_counter_(0), remove_targets_(false)
{
	//set random seed for the rng
	rand_seed();

}

Cscene_sim::~Cscene_sim(  )
{
	for( std::list<Cperson_abstract*>::iterator iit = person_list_.begin() ; iit != person_list_.end() ; iit++ )
	{
		delete *iit;
	}
	delete robot_;
}


Cperson_abstract* Cscene_sim::add_person_container( unsigned int id,
		std::list<Cperson_abstract*>::iterator iit)
{
	Cperson_virtual* person = new Cperson_virtual(id, Cperson_abstract::Person, scene_force_type_);
	person_list_.insert( iit , person );
	//person_list_.insert( iit , (Cperson_abstract*) person );
	person->set_destinations(destinations_);
	return (Cperson_abstract*) person;
}

void Cscene_sim::update_scene(const std::vector<SdetectionObservation>& observation )
{
	//the filter variable is not used in the simulated scene. All observations have been filtered.
	double now = observation[0].time_stamp;
	std::list<Cperson_abstract*>::iterator iit;

	//deleting persons if needed, (LIFO)
	while( person_list_.size() > number_virtual_people_ )
	{
		person_list_.pop_front();
	}

	//updating persons trajectories
	double w;
	SpointV_cov virtual_next_pose;
	Sforce virtual_force_goal, virtual_force_int_person, virtual_force_obstacle, virtual_force_robot;
	for( iit = person_list_.begin() ; iit!=person_list_.end() ; iit++)
	{
		if ( (*iit)->get_person_type() == Cperson_abstract::Person )
		{
			//1 time step propagation
			virtual_force_goal = (*iit)->force_goal( (*iit)->get_best_dest(), this->get_sfm_params(*iit)  );
			virtual_force_int_person = force_persons_int( *iit );
			virtual_force_obstacle = get_force_map( (*iit)->get_current_pointV().x, (*iit)->get_current_pointV().y ) * 0.5;
			// calculate the robot-person force specifically, if any robot in the scene
			if ( robot_in_the_scene_ )
			{
				virtual_force_robot =  (*iit)->force( robot_->get_current_pointV() ,
					this->get_sfm_int_params(*iit, robot_)   ) * 4;//TODO change parameters
			}
			else{
				virtual_force_robot = Sforce();
			}
			//to avoid entering obstacles present in the scene, it simply does not propagate
			w = 1.0;
			do
			{
				(*iit)->set_forces_person( virtual_force_goal*w, virtual_force_int_person*w, virtual_force_robot*w, virtual_force_obstacle );
				virtual_next_pose = (*iit)->pointV_propagation( dt_ , (*iit)->get_force_person()  );
				w *= 0.9;
			} while( read_force_map_success_ && !is_cell_clear_map( virtual_next_pose.x, virtual_next_pose.y ) && w > 0.1 );
			virtual_next_pose.time_stamp = now;
			(*iit)->add_pointV( virtual_next_pose );
		}
	}

	//generating virtual persons, if necessary
	Cperson_abstract* new_person;
	Sdestination dest_1, dest_2;
	while( person_list_.size() < number_virtual_people_ )
	{
		// TODO check size of destinations > 0
		new_person = add_person( ++virtual_person_id_counter_);
		//set initial position at random. initially 2 destinations
		dest_1 = get_destinations()->at( rand_uniform_discrete( 0, (int)get_destinations()->size()-1 ) );
		dest_2 = get_destinations()->at( rand_uniform_discrete( 0, get_destinations()->size()-1) );
		while( dest_1 == dest_2 && get_destinations()->size() > 1 )
			dest_2 = get_destinations()->at( rand_uniform_discrete( 0, get_destinations()->size()-1) );
		//initial pose checks for free cells if an obstacle map is provided
		double x, y;
		do{
			double lambda_x = rand_uniform( 0, 1.0 ) , lambda_y = rand_uniform( 0, 1.0 );
			x =lambda_x*dest_1.x + (1-lambda_x)*dest_2.x + rand_normal(0,1.5);
			y = lambda_y*dest_1.y + (1-lambda_y)*dest_2.y + rand_normal(0,1.5);
		} while( read_force_map_success_ && !is_cell_clear_map( x, y ) );
		new_person->add_pointV( SpointV_cov(x , y, now),  filtering_method_ );
		//set destination at random
		new_person->set_desired_velocty( rand_normal(1.0,0.1) );
		new_person->set_best_dest( new_person->get_destinations()->at( rand_uniform_discrete( 0, new_person->get_destinations()->size()-1)  ) );
		//new_person->print();
	}

	//deleting complete trajectories
	if ( remove_targets_ )
	{
		std::vector<unsigned int> targets_to_remove;
		for(iit = person_list_.begin() ; iit!=person_list_.end() ; iit++)
		{
			if ( (*iit)->get_person_type() == Cperson_abstract::Person &&
					(*iit)->get_current_pointV().distance( (Spoint)(*iit)->get_best_dest() ) < 1.5 )//destination achieved
			{
				targets_to_remove.push_back((*iit)->get_id());
				number_virtual_people_--;
			}
		}
		for ( unsigned int i = 0; i < targets_to_remove.size() ; ++i)
			remove_person(targets_to_remove[i]);
	}
	//assigning new destinations once the goal is achieved
	else
	{
		std::vector<unsigned int> targets_to_update;
		for(iit = person_list_.begin() ; iit!=person_list_.end() ; iit++)
		{
			if ( (*iit)->get_person_type() == Cperson_abstract::Person &&
					(*iit)->get_current_pointV().distance( (Spoint)(*iit)->get_best_dest() ) < 1.5 )//destination achieved
			{
				//set destination at random
				(*iit)->set_desired_velocty( rand_normal(1.0,0.1) );
				std::vector<int> neighbours = (*iit)->get_best_dest().neighbours_ids;
				int new_dest = neighbours[ rand_uniform_discrete( 0, neighbours.size()-1 )];
				(*iit)->set_best_dest( destinations_[ new_dest ] );
			}
		}
	}
}


void Cscene_sim::clear_scene()
{
	robot_->reset();
	for( std::list<Cperson_abstract*>::iterator iit = person_list_.begin() ; iit != person_list_.end() ; iit++ )
	{
		delete *iit;
	}
	 person_list_.clear();
}
