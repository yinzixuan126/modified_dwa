/*
 * test_Cperson.cpp
 *
 * A test for the scene type elements. TODO
 *
 *  Created on: Oct 1, 2013
 *      Author: gferrer
 */

#define BOOST_TEST_MODULE test_scene
#include <boost/test/unit_test.hpp>
//#include "scene_elements/person_abstract.h"
//#include "scene_elements/person_virtual.h"
//#include "scene_elements/person_bhmip.h"
#include "scene_abstract.h"
#include "prediction_bhmip.h"
#include "prediction_behavior.h"
#include "scene_sim.h"

//Alias in order to easily validate a new scene type

BOOST_AUTO_TEST_CASE( real_scene )
{
	typedef Cprediction_bhmip T;
	// testing constructor, no destinations neither map set
	T scene;
	scene.set_dt(0.1);
	std::vector<SdetectionObservation> obs_scene;
	obs_scene.push_back( SdetectionObservation(1, 0.0) );
    Cperson_abstract* person;

    //test propagation, 10 time steps
    for( unsigned int i = 0; i<=10; ++i)
    {
		obs_scene[0].x = (double)i ;
		obs_scene[0].time_stamp = (double)i * 0.1;
		scene.update_scene( obs_scene );
		if( scene.find_person(1,&person))
			person->get_current_pointV().print();
    }
    //check find person
    BOOST_CHECK( scene.find_person(1) );
    BOOST_CHECK( !scene.find_person(7474) );
    BOOST_CHECK( scene.find_person(1,&person) );

    //check propagation
    BOOST_CHECK_MESSAGE( person->get_current_pointV().distance( Spoint(10.0,0.0) ) < 0.05 ,
    		"distance from current to Spoint(10,0) = " << person->get_current_pointV().distance( Spoint(10.0,0.0) ));
    SpointV_cov prop = person->pointV_propagation( 0.1 );
    prop.print();
    //check if propagation is done correctly, error 0.5 m
    BOOST_CHECK_MESSAGE( prop.distance( Spoint( 11,0 ) ) < 0.5 ,
     		"distance from current to Spoint(11,0) = " << prop.distance( Spoint(11.0,0.0) ));

    //check for size of container
	const std::list<Cperson_abstract *>* person_list = scene.get_scene( );
    BOOST_CHECK( person_list->size() == 1 );

    //adding extra persons
    obs_scene.push_back( SdetectionObservation(2, 0.0) );
	scene.update_scene( obs_scene );
	person_list = scene.get_scene( );
	BOOST_CHECK( person_list->size() == 2 );
	BOOST_CHECK( person_list->front()->get_id() == 1 );
	BOOST_CHECK( person_list->back()->get_id() == 2 );

    //removing persons
	obs_scene.clear();
    obs_scene.push_back( SdetectionObservation(2, 0.0) );
    scene.update_scene( obs_scene );
	person_list = scene.get_scene( );
	BOOST_CHECK( person_list->size() == 1 );
    BOOST_CHECK( scene.find_person(2) );
    BOOST_CHECK( !scene.find_person(1) );

    //ordering persons
	obs_scene.clear();
	obs_scene.push_back( SdetectionObservation(4, 0.0) );
	obs_scene.push_back( SdetectionObservation(3, 0.0) );
    obs_scene.push_back( SdetectionObservation(2, 0.0) );
    scene.update_scene( obs_scene );
	person_list = scene.get_scene( );
	BOOST_CHECK( person_list->front()->get_id() == 2 );
	BOOST_CHECK( person_list->back()->get_id() == 4 );

    scene.update_scene( obs_scene );
	person_list = scene.get_scene( );
	BOOST_CHECK( person_list->size() == 3 );
    BOOST_CHECK( scene.find_person(2) );
    BOOST_CHECK( scene.find_person(3) );
    BOOST_CHECK( scene.find_person(4) );
	BOOST_CHECK( person_list->front()->get_id() == 2 );
	BOOST_CHECK( person_list->back()->get_id() == 4 );

    //read destination and map
    BOOST_CHECK( scene.read_destination_map(
    		"/home/gferrer/iri-lab/ros/iri-ros-pkg/iri_navigation/iri_people_simulation/map/map_destinations_learning.txt"  ) );
    BOOST_CHECK( scene.read_force_map(
    		"/home/gferrer/iri-lab/ros/iri-ros-pkg/iri_navigation/iri_people_simulation/map/force_map_sim.txt" ));
}

BOOST_AUTO_TEST_CASE( sim_scene )
{


}
