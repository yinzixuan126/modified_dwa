/*
 * prediction_example.cpp
 *
 *  Created on: 15-Apr-2014
 *      Author: gferrer
 */

#include "prediction_behavior.h"
#include "scene_sim.h"
#include <iostream>

int main(int argc,char *argv[])
{
	Cscene_sim scene;
	scene.set_dt(0.1);
	scene.set_number_virtual_people( 2 );
	if ( !scene.read_destination_map(
		"/home/gferrer/iri-lab/ros/iri-ros-pkg/iri_navigation/iri_people_simulation/map/3_destinations.txt"  ) )
	{
		std::cout << "Could not read map destinations file !!!" << std::endl;
	}
	else{
		std::cout << "read destinations map file : SUCCESS!!!" << std::endl;
	}
	scene.set_remove_targets(false);

	// prediction ( double horizon_time = 0.5, bool behavior_flag = false,filter,autoremove )
	Cprediction_behavior planner(1.0, true, Cperson_abstract::Low_pass_linear_regression_filtering);
	if ( !planner.read_destination_map(
		"/home/gferrer/iri-lab/ros/iri-ros-pkg/iri_navigation/iri_people_simulation/map/3_destinations.txt"  ) )
	{
		std::cout << "Could not read map destinations file !!!" << std::endl;
	}
	else{
		std::cout << "read destinations map file : SUCCESS!!!" << std::endl;
	}
	planner.set_dt(0.1);


	double now = 0.0;

	std::vector<SdetectionObservation> obs_scene;
	while(now < 100.0)
	{
		std::cout << "entering loop sequence at time " << now << std::endl;
		obs_scene.clear();
		obs_scene.push_back( SdetectionObservation(0, now ));//void observation, just for the timestamp
		scene.update_scene( obs_scene );

		//update planner
		obs_scene.clear();
		const std::list<Cperson_abstract *>* person_list = scene.get_scene(  );
		for( Cperson_abstract* iit : *person_list)
		{
			obs_scene.push_back( SdetectionObservation( iit->get_id(), now,
					iit->get_current_pointV().x , iit->get_current_pointV().y) );
		}
		planner.update_scene( obs_scene );
		planner.update_robot( Spose(0.0 , 0.0 , now  ) );

		//calculation of prediction
		planner.scene_prediction();

		// testing of predictions

		now +=0.1;
	}

	return 0;
}

