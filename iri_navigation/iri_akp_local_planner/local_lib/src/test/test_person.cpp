/*
 * test_Cperson.cpp
 *
 * TODO complete the test, it was done after the designe of the Cpersons classes
 *
 *  Created on: Oct 1, 2013
 *      Author: gferrer
 */

#define BOOST_TEST_MODULE Cperson_test
#include <boost/test/unit_test.hpp>
#include "scene_elements/person_abstract.h"
#include "scene_elements/person_virtual.h"
#include "scene_elements/person_bhmip.h"
#include "scene_elements/person_behavior.h"

//Alias in order to easily validate a new type of person
typedef Cperson_behavior T;
//typedef Cperson_virtual T;

BOOST_AUTO_TEST_CASE( construction_and_setting )
{
	// testing constructor to created a person with id
	unsigned int id = 0;
    Cperson_abstract* person = new T( id );
    BOOST_CHECK( person->get_id() == id );
    delete person;

    // testing if construction creates person of type Person
    person = new T( id, Cperson_abstract::Person );
    BOOST_CHECK( person->get_id() == id && person->get_person_type () == Cperson_abstract::Person);

    // testing if construction creates person of type Robot
    Cperson_abstract* person2 = new T( 1, Cperson_abstract::Robot , Cperson_abstract::Spherical);
    BOOST_CHECK( person2->get_id() == 1 && person2->get_person_type () == Cperson_abstract::Robot);

    // testing if operator != works fine (only for classes, not pointers!): different id
    BOOST_CHECK( *person != *person2 );

    // testing if operator != works fine: same id but different type
	delete person2;
	person2 = new T( id, Cperson_abstract::Robot );
    BOOST_CHECK( *person != *person2 );

    // testing if operator == works fine on itself
    BOOST_CHECK( *person == *person );

    // testing if operator == works fine: same id, same type but different force type
    delete person2;
    person2 = new T( id, Cperson_abstract::Person , Cperson_abstract::Spherical);
    BOOST_CHECK( *person == *person2 );

    delete person;
    delete person2;
}

BOOST_AUTO_TEST_CASE( prediction )
{
    Cperson_abstract* person = new T( 0 );

    // setting of destinations
    std::vector<Sdestination> dests;
    dests.push_back( Sdestination(1,10,0,0.25) );
    dests.push_back( Sdestination(2,10,-10,0.25) );
    dests.push_back( Sdestination(3,-10,-10,0.25) );
    dests.push_back( Sdestination(4,-10,-10,0.25) );
    person->set_destinations( dests );
    BOOST_CHECK( person->get_destinations()->size() == 4 );

    //propagation
    person->add_pointV(SpointV_cov(0,0,0),Cperson_abstract::Linear_regression_filtering);
    person->add_pointV(SpointV_cov(0.1,0,0.1),Cperson_abstract::Linear_regression_filtering);
    person->add_pointV(SpointV_cov(0.2,0,0.2),Cperson_abstract::Linear_regression_filtering);
    BOOST_CHECK( person->get_current_pointV().propagate(0.1).distance( Spoint(0.3,0) ) < 0.05 );


    delete person;
}
