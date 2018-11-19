/*
 * test_Cperson.cpp
 *
 * TODO complete the test, it was done after the designe of the Cpersons classes
 *
 *  Created on: Oct 1, 2013
 *      Author: gferrer
 */

#define BOOST_TEST_MODULE geometry_test
#include <boost/test/unit_test.hpp>
#include "iri_geometry.h"


BOOST_AUTO_TEST_CASE( point )
{
    // testing if construction creates Spoint by default
	Spoint p;
    BOOST_CHECK( p.x == 0 && p.y == 0 && p.time_stamp == 0 );

    Spoint p2 = Spoint( 1.0, 3.0, 54.3);
	BOOST_CHECK( p2.x == 1.0 && p2.y == 3.0 && p2.time_stamp == 54.3 );

	p = p2;
	BOOST_CHECK( p.x == 1.0 && p.y == 3.0 && p.time_stamp == 54.3 );

	Spoint p3 = p2-p;
    BOOST_CHECK( p3.x == 0 && p3.y == 0 && p3.time_stamp == 54.3 );

    p2 = p2 * (-1);
    p3 = p2 + p;
    BOOST_CHECK( p3.x == 0 && p3.y == 0 && p3.time_stamp == 54.3 );

    p = Spoint( 5,0 );
    BOOST_CHECK( p.distance( Spoint() ) == 5.0  );
    BOOST_CHECK( p.distance( ) == 5.0  );

    p = Spoint( 3,4 );
	BOOST_CHECK( p.distance( Spoint() ) == 5.0  );
	BOOST_CHECK( p.distance( ) == 5.0  );

	p2 = p.propagate(0.1);
	BOOST_CHECK( p2.distance(p) == 0.0  );

	p2 = p.propagate(0.1,Sforce(), 1.0);
	BOOST_CHECK( p2.distance(p) == 0.0  );
}


BOOST_AUTO_TEST_CASE( point_cov )
{
   // testing if construction creates Spoint by default
	Spoint_cov p;
	BOOST_CHECK( p.x == 0 && p.y == 0 && p.time_stamp == 0 );
	BOOST_CHECK( p.cov[0] == 1.0 && p.cov[1] == 0.0 &&  p.cov[2] == 0 && p.cov[3] == 1.0);

	p = Spoint_cov(1.0,2.0,0.0);
	BOOST_CHECK( p.x == 1.0 && p.y == 2.0 && p.time_stamp == 0.0 );
	BOOST_CHECK( p.cov[0] == 1.0 && p.cov[1] == 0 &&  p.cov[2] == 0.0 && p.cov[3] == 1.0);

	//pass invalid covariance matrix (<4)
	std::vector<double> cov(3,0.0);
	p = Spoint_cov(1.0,2.0,0.0,cov);
	BOOST_CHECK( p.x == 1.0 && p.y == 2.0 && p.time_stamp == 0.0 );
	BOOST_CHECK( p.cov[0] == 1.0 && p.cov[1] == 0 &&  p.cov[2] == 0.0 && p.cov[3] == 1.0);

	cov.resize(4,0.0);
	cov[0]=3.0;
	p = Spoint_cov(1.0,1.0,0.0,cov);
	BOOST_CHECK( p.x == 1.0 && p.y == 1.0 && p.time_stamp == 0.0 );
	BOOST_CHECK( p.cov[0] == 3.0 && p.cov[1] == 0 &&  p.cov[2] == 0.0 && p.cov[3] == 0);

	//operators
	Spoint_cov p2 = Spoint_cov(1.0,-1.0,0.0,cov);
    Spoint_cov p3 = p2 + p;
    BOOST_CHECK( p3.x == 2.0 && p3.y == 0.0 && p3.time_stamp == 0.0 );
    BOOST_CHECK( p3.cov[0] == 3.0 && p3.cov[1] == 0 &&  p3.cov[2] == 0.0 && p3.cov[3] == 0);

    p3 = p - p;
    BOOST_CHECK( p3.x == 0.0 && p3.y == 0.0 && p3.time_stamp == 0.0 );
    BOOST_CHECK( p3.cov[0] == 3.0 && p3.cov[1] == 0 &&  p3.cov[2] == 0.0 && p3.cov[3] == 0);

    p = p*2.5;
    BOOST_CHECK( p.x == 2.5 && p.y == 2.5 && p.time_stamp == 0.0 );


	//distances
    p = Spoint_cov(2.0,4.0,.0,cov);
	p2 = Spoint_cov( 5,0 );
    BOOST_CHECK( p2.distance( p ) == 5.0  );
    Spoint p4 = (Spoint)p;
    BOOST_CHECK( p2.distance2( p4 ) == 25.0  );
    BOOST_CHECK( p2.distance( ) == 5.0  );

    //covariances distances: default identity
    BOOST_CHECK( p2.cov_dist( p ) == 25.0  );
    BOOST_CHECK( p2.cov_dist( Spoint() ) == 25.0  );
    BOOST_CHECK( p2.cov_dist(  ) == 25.0  );

    //covariances distances: matrix cov
    p2.cov[0]=2.0;
    p2.cov[3]=2.0;
    BOOST_CHECK( p2.cov_dist( p ) == 12.5  );
    BOOST_CHECK( p2.cov_dist( Spoint() ) == 12.5  );
    BOOST_CHECK( p2.cov_dist(  ) == 12.5  );


}

BOOST_AUTO_TEST_CASE( point_velocity )
{
    // testing if construction creates Spoint by default
	SpointV p;
    BOOST_CHECK( p.x == 0 && p.y == 0 && p.vx == 0 && p.vy == 0 && p.time_stamp == 0 );

    SpointV p2 = SpointV( 1.0, 3.0, 54.3, 1.0, -1.0);
	BOOST_CHECK( p2.x == 1.0 && p2.y == 3.0 );
	BOOST_CHECK( p2.vx == 1.0 && p2.vy == -1.0);
	BOOST_CHECK( p2.time_stamp == 54.3 );

	p = p2;
	BOOST_CHECK( p.x == 1.0 && p.y == 3.0 );
	BOOST_CHECK( p.vx == 1.0 && p.vy == -1.0);
	BOOST_CHECK( p.time_stamp == 54.3 );

	SpointV p3 = p2-p;
	BOOST_CHECK( p3.x == 0.0 && p3.y == 0.0 );
	BOOST_CHECK( p3.vx == 0.0 && p3.vy == 0.0);
	BOOST_CHECK( p3.time_stamp == 0.0 );

    p2 = p2 * (-1);
    //p2.Spoint::print();
    //p2.print();
    p3 = p2 + p;
	BOOST_CHECK( p3.x == 0.0 && p3.y == 0.0 );
	BOOST_CHECK( p3.vx == 0.0 && p3.vy == 0.0);
	BOOST_CHECK( p3.time_stamp == 54.3 );

    p = SpointV( 5,0 );
    BOOST_CHECK( p.distance( SpointV() ) == 5.0  );
    BOOST_CHECK( p.distance( ) == 5.0  );

    p = SpointV( 3,4,0, 1,0 );
	BOOST_CHECK( p.distance( SpointV() ) == 5.0  );
	BOOST_CHECK( p.distance( ) == 5.0  );


	p2 = p.propagate(1);
	BOOST_CHECK( p2.x == 4 && p2.y == 4  );

	p2 = p.propagate(1, Sforce(), 0.5);
	BOOST_CHECK( p2.x == 3.5 && p2.y == 4  );

	p2 = p.propagate(1, Sforce(1,0), 5.0 );
	//p2.print();
	BOOST_CHECK( p2.x == 4.5 && p2.vx == 2  );

    //compatibility to other geometry classes
    BOOST_CHECK( p.distance( Spoint() ) == 5.0  );
}
BOOST_AUTO_TEST_CASE( point_velocity_covariance )
{
	// constructor by default
	SpointV_cov p;
	BOOST_CHECK( p.x == 0 && p.y == 0 && p.time_stamp == 0 && p.vx == 0.0 &&p.vy==0.0);
	BOOST_CHECK( p.cov[0] == 0.4 && p.cov[1] == 0 &&  p.cov[2] == 0 && p.cov[3] == 0 &&
			     p.cov[4] == 0 && p.cov[5] == 0.4 &&  p.cov[6] == 0 && p.cov[6] == 0 &&
			     p.cov[8] == 0 && p.cov[9] == 0 &&  p.cov[10] == 0.1 && p.cov[11] == 0 &&
			     p.cov[12] == 0 && p.cov[13] == 0 &&  p.cov[14] == 0 && p.cov[15] == 0.1);

	p = SpointV_cov(1.0,3.0,23.8,1.0,2.3);
	BOOST_CHECK( p.x == 1.0 && p.y == 3.0 && p.time_stamp == 23.8 && p.vx == 1.0 &&p.vy==2.3);
	BOOST_CHECK( p.cov[0] == 0.4 && p.cov[1] == 0 &&  p.cov[2] == 0 && p.cov[3] == 0 &&
			     p.cov[4] == 0 && p.cov[5] == 0.4 &&  p.cov[6] == 0 && p.cov[6] == 0 &&
			     p.cov[8] == 0 && p.cov[9] == 0 &&  p.cov[10] == 0.1 && p.cov[11] == 0 &&
			     p.cov[12] == 0 && p.cov[13] == 0 &&  p.cov[14] == 0 && p.cov[15] == 0.1);

	//invalid covariance
    std::vector<double> cov(3,0.0);
    p = SpointV_cov(1.0,3.0,23.8,1.0,2.3,cov);
    BOOST_CHECK( p.x == 1.0 && p.y == 3.0 && p.time_stamp == 23.8 && p.vx == 1.0 &&p.vy==2.3);
	BOOST_CHECK( p.cov[0] == 0.4 && p.cov[1] == 0 &&  p.cov[2] == 0 && p.cov[3] == 0 &&
			     p.cov[4] == 0 && p.cov[5] == 0.4 &&  p.cov[6] == 0 && p.cov[6] == 0 &&
			     p.cov[8] == 0 && p.cov[9] == 0 &&  p.cov[10] == 0.1 && p.cov[11] == 0 &&
			     p.cov[12] == 0 && p.cov[13] == 0 &&  p.cov[14] == 0 && p.cov[15] == 0.1);

    //valid covariance
    cov.resize(16,0.0);
    cov[0]=1.0;
    cov[5]=10;
    p = SpointV_cov(1.0,1.0,23.8,1.0,-1.0,cov);
    BOOST_CHECK( p.cov[0] == 1.0 && p.cov[1] == 0 &&  p.cov[2] == 0 && p.cov[3] == 0 &&
                 p.cov[4] == 0 && p.cov[5] == 10.0 &&  p.cov[6] == 0 && p.cov[6] == 0 &&
                 p.cov[8] == 0 && p.cov[9] == 0 &&  p.cov[10] == 0.0 && p.cov[11] == 0 &&
                 p.cov[12] == 0 && p.cov[13] == 0 &&  p.cov[14] == 0 && p.cov[15] == 0.0);

    SpointV pp(1.0,1.0,23.8,1.0,-1.0);
    p = SpointV_cov( pp, cov );
    BOOST_CHECK( p.x == 1.0 && p.y == 1.0 && p.time_stamp == 23.8 && p.vx == 1.0 &&p.vy==-1.0);
    BOOST_CHECK( p.cov[0] == 1.0 && p.cov[1] == 0 &&  p.cov[2] == 0 && p.cov[3] == 0 &&
                     p.cov[4] == 0 && p.cov[5] == 10.0 &&  p.cov[6] == 0 && p.cov[6] == 0 &&
                     p.cov[8] == 0 && p.cov[9] == 0 &&  p.cov[10] == 0.0 && p.cov[11] == 0 &&
                     p.cov[12] == 0 && p.cov[13] == 0 &&  p.cov[14] == 0 && p.cov[15] == 0.0);


    //operators
    SpointV_cov p2 = p;
    SpointV_cov p3 = p2-p;
    BOOST_CHECK( p3.x == 0.0 && p3.y == 0.0 );
    BOOST_CHECK( p3.vx == 0.0 && p3.vy == 0.0);
    BOOST_CHECK( p3.time_stamp == 0.0 );

    p2 = p2 * (-1);
    //p2.Spoint::print();
    //p2.print();
    p3 = p2 + p;
    BOOST_CHECK( p3.x == 0.0 && p3.y == 0.0 );
    BOOST_CHECK( p3.vx == 0.0 && p3.vy == 0.0);
    BOOST_CHECK( p3.time_stamp == 23.8 );

    //check propagation
    p = SpointV_cov(1.0,1.0,23.8,1.0,-1.0);
    p = p.propagate( 1.0 );

}
