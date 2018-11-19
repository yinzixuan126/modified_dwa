/*
 * local_nav_example.cpp
 *
 *  Created on: 10-Jan-2014
 *      Author: gferrer
 */

#include "nav/plan_local_nav.h"
#include "scene_sim.h"
#include <iostream>

int main(int argc,char *argv[])
{
	Cscene_sim scene;
	double dt(0.2);
	scene.set_dt(dt);
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

	// planner
	Cplan_local_nav planner(5.0,200);
	planner.set_global_mode( Cplan_local_nav::MO_norm );
	planner.set_distance_mode( Cplan_local_nav::Cost2go_norm );
	//Cprediction_behavior planner;
	if ( !planner.read_destination_map(
		"/Xhome/gferrer/iri-lab/ros/iri-ros-pkg/iri_navigation/iri_people_simulation/map/3_destinations.txt"  ) )
	{
		std::cout << "Could not read map destinations file !!!" << std::endl;
	}
	else{
		std::cout << "read destinations map file : SUCCESS!!!" << std::endl;
	}
	planner.set_dt(dt);


	double now = 10.0;

	std::vector<SdetectionObservation> obs_scene;
	std::vector<Spoint> laser_points;
	laser_points.push_back( Spoint(0.9,0.9) );//poruqe? TODO
	Spose command;

	while(now < 100.0)
	{
		std::cout << "entering loop sequence at time " << now << std::endl;
		obs_scene.clear();
		obs_scene.push_back( SdetectionObservation(0, now ));//void observation, just for the timestamp
		if (now > 0.2)
			scene.set_number_virtual_people( 1);
		scene.update_scene( obs_scene );
		scene.update_robot( Spose(0.0 , 0.0 , now  ) );

		//update planner
		obs_scene.clear();
		const std::list<Cperson_abstract *>* person_list = scene.get_scene(  );
		for( Cperson_abstract* iit : *person_list)
		{
			iit->get_current_pointV().print();
			obs_scene.push_back( SdetectionObservation( iit->get_id(), now,
					iit->get_current_pointV().x , iit->get_current_pointV().y) );
		}
		planner.update_scene( obs_scene );
		planner.update_robot( Spose(0.0 , 0.0 , now  ) );

		planner.read_laser_scan( laser_points );

		//calculation of
		planner.set_robot_goal( Spoint(5,5) );
		planner.robot_plan(command);

		now +=dt;
	}

	return 0;
}

