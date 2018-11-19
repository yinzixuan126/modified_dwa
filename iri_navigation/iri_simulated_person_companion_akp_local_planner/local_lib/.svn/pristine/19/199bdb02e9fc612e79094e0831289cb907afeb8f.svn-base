#include "scene_elements/person_abstract.h"
#include <list>
#include <math.h>
#include <algorithm>
#include <iostream>



Cperson_abstract::Cperson_abstract(  unsigned int id , Cperson_abstract::target_type target_type,
		Cperson_abstract::force_type person_force_type, double _time_window):
	person_id_( id ) ,
	desired_velocity_(0.0),
	type_(target_type),
	person_force_type_(person_force_type),
	observation_update_(false),
	now_(0.0)
{


}

Cperson_abstract::~Cperson_abstract()
{

}

bool Cperson_abstract::operator== (Cperson_abstract& p2) const
{
	if ( person_id_ == p2.get_id() && type_ == p2.get_person_type() )
		return true;
	else
		return false;
}

//TODO add method for implicitly calculating force depending on prediction if none is provided
SpointV_cov Cperson_abstract::pointV_propagation( double dt , Sforce force )
{
	return current_pointV_.propagate( now_- current_pointV_.time_stamp + dt , force , desired_velocity_);
}

SpointV_cov Cperson_abstract::pointV_propagation( double dt , Sforce force, SpointV_cov point )
{
	return point.propagate( now_- current_pointV_.time_stamp + dt,force, desired_velocity_ );
}


void Cperson_abstract::print()
{
	std::cout << std::endl << "              last pose " << person_id_ << "th print :" << std::endl;
	current_pointV_.print();
	return;
}

void Cperson_abstract::print_dest()
{
	std::cout << std::endl << "                  Best destination of person " << person_id_ << std::endl;
	best_destination_.print();
}

Sforce Cperson_abstract::force_ellip( const SpointV& interacting_person ,  const std::vector<double>* social_forces_param, const SpointV* virtual_current_point )
{

	//if force is calculated from a propagated pose, then virtual_current_point should be pointing to a valid SpointV.
	//Otherwise, current point is current point class variable at time "now"
	const SpointV* current_point;
	if ( virtual_current_point == NULL  )
		current_point = &current_pointV_;
	else
		current_point = virtual_current_point;

	//setting the corresponding SFM parameters:
	// if no forces param provided, exit program
	assert( social_forces_param != NULL );
	assert( social_forces_param->size() >= 5 );

	Sforce force;
	//geometry calculations
	double tau = social_forces_param->at(4);
	SpointV pose_dif = (SpointV)(*current_point) - interacting_person;
	Spoint d_point( pose_dif.x,pose_dif.y );
	Spoint v_current = Spoint( current_point->vx , current_point->vy );
	Spoint v_int = Spoint( interacting_person.vx , interacting_person.vy );
	//Spoint y_point = d_point - (v_int - v_current )*tau;
	//force module calculation
	double b = d_point.distance() + (d_point - ( v_int - v_current )*tau ).distance();
	b = 0.5* sqrt( ( b*b - (( v_int - v_current )*tau ).distance() * (( v_int - v_current )*tau ).distance()
			) / ( 1 + v_current.distance() * tau ) );
	double f = social_forces_param->at(2)* exp( (- b) / social_forces_param->at(3));
	f = f * (d_point.distance() + (d_point - ( v_int - v_current )*tau ).distance()) / sqrt( 1 + v_current.distance() * tau ) / (4*b);
	//force direction
	double phi = current_point->angle_heading_point( interacting_person );
	double anisotropy = (social_forces_param->at(1) + (1-social_forces_param->at(1))*(1 + cos(phi))/2 );
	double dx,dy;
	if ( (( v_int - v_current )*tau).distance() > 0.01 )
	{
	dx = d_point.x/d_point.distance() + (( v_int - v_current )*tau).x / (( v_int - v_current )*tau).distance() ;
	dy = d_point.y/d_point.distance() + (( v_int - v_current )*tau).y / (( v_int - v_current )*tau).distance() ;
	}
	else
	{
		dx = d_point.x/d_point.distance();
		dy = d_point.y/d_point.distance();
	}
	force.fx = f * dx * anisotropy;
	force.fy = f * dy * anisotropy;
	return force;
}

Sforce Cperson_abstract::force_cp( const SpointV& interacting_person ,  const std::vector<double>* social_forces_param, const SpointV* virtual_current_point )
{
	//if force is calculated from a propagated pose, then virtual_current_point won't be null.
	//Otherwise, current point is current point class variable
	const SpointV* current_point;
	if ( virtual_current_point == NULL  )
		current_point = &current_pointV_;
	else
		current_point = virtual_current_point;

	//setting the corresponding SFM parameters:
	// if no forces param provided, exit program
	assert( social_forces_param != NULL );
	assert( social_forces_param->size() >= 5 );

	Sforce force;
	//obtaining time of collision prediction
	//t = - (dx*dvx + dy * dvy ) / (dvx^2 + dvy^2)
	double dx = current_point->x - interacting_person.x;
	double dy = current_point->y - interacting_person.y;
	double vx = current_point->vx;
	double vx_i = interacting_person.vx;
	double dvx =  vx - vx_i;
	double vy = current_point->vy;
	double vy_i = interacting_person.vy;
	double dvy = vy - vy_i;
	double time_minimum_distance = - (dx*dvx + dy*dvy ) / (dvx*dvx + dvy*dvy);

	//propagating state-
	double dx_prop = current_point->x + time_minimum_distance * vx -
			interacting_person.x - time_minimum_distance * vx_i;
	double dy_prop = current_point->y + time_minimum_distance * vy -
			interacting_person.y - time_minimum_distance * vy_i;
	double d_cp = dx_prop * dx_prop + dy_prop * dy_prop;

	//exponential force module f = A*v/t* exp(d_cp / B)
	double f;
	if ( time_minimum_distance > 0  &&
		fabs(diffangle(diffangle(current_point->orientation(),interacting_person.orientation()) , atan2( dvy , dvx ) )) < PI/2 )
	{
		if (time_minimum_distance < 0.2 ) time_minimum_distance = 0.2;
		if (d_cp < 0.36)
		{
			dx_prop = dx_prop / d_cp * 0.36;//divide mal... too bad a corregir mañana
			dy_prop = dy_prop / d_cp * 0.36;
			d_cp = 0.36;//0.3 m radii per person
		}
		f = social_forces_param->at(2)*current_point->v() / time_minimum_distance * exp(- d_cp / social_forces_param->at(3));
		//force direction
		double phi = diffangle( atan2(vy,vx), atan2(dy_prop,dx_prop) );
		double anisotropy =  (social_forces_param->at(1) + (1-social_forces_param->at(1))*(1 + cos(phi))/2 );
		force.fx = f * dx_prop * anisotropy;
		force.fy = f * dy_prop * anisotropy;
	}

	return force;
}

Sforce Cperson_abstract::force_sphe( const Spoint& interacting_person ,  const std::vector<double>* social_forces_param , const SpointV* virtual_current_point )
{
	//if force is calculated from a propagated pose, then virtual_current_point won't be null.
	//Otherwise, current point is current point class variable
	const SpointV* current_point;
	if ( virtual_current_point == NULL  )
		current_point = &current_pointV_;
	else
		current_point = virtual_current_point;

	//setting the corresponding SFM parameters:
	// if no forces param provided, exit program
	assert( social_forces_param != NULL );
	assert( social_forces_param->size() >= 5 );

	Sforce force;
	double f,d;
	//geometry calculations
	double dx = current_point->x - interacting_person.x;
	double dy = current_point->y - interacting_person.y;
	double vx = current_point->vx;
	double vy = current_point->vy;
	d = sqrt(dx*dx+dy*dy);
	dx /= d;
	dy /= d;
	//force module calculation
	f = social_forces_param->at(2)* exp((social_forces_param->at(4)-d) / social_forces_param->at(3));
	//force direction
	double phi = diffangle( atan2(vy,vx), atan2(-dy,-dx) );//minus difference vector
	double anisotropy = (social_forces_param->at(1) + (1-social_forces_param->at(1))*(1 + cos(phi))/2 );
	force.fx = f * dx * anisotropy;
	force.fy = f * dy * anisotropy;
	return force;
}


Sforce Cperson_abstract::force( const SpointV& interacting_person ,  const std::vector<double>* social_forces_param, const SpointV* virtual_current_point )
{
	Sforce force_res;
	assert( !social_forces_param->empty() );
	switch(person_force_type_)
	{
	case Elliptical:
		force_res = force_ellip(  interacting_person ,  social_forces_param, virtual_current_point  );
		break;
	case Collision_Prediction:
		force_res = force_cp(  interacting_person ,  social_forces_param, virtual_current_point );
		break;
	case Spherical:
	default:
		force_res = force_sphe(  interacting_person ,  social_forces_param, virtual_current_point  );
		break;
	}
	return force_res;
}

Sforce Cperson_abstract::force_goal(  const Sdestination& dest, const std::vector<double>* social_forces_param, const SpointV* virtual_current_point )
{
	//if there are no is inferred, then no force to goal is possible (linear propagation + interaction forces)
	if ( dest.type == Sdestination::Uncertain  ) return Sforce();

	//if force is calculated from a propagated pose, then index would be different form zero and
	// the prediction vector should not be empty
	const SpointV* current_point;
	if ( virtual_current_point == NULL )
		current_point = &current_pointV_;
	else
		current_point = virtual_current_point;

	//setting the target type and its corresponding parameters:
	// if no forces param provided, exit program
	assert( social_forces_param != NULL );
	assert( social_forces_param->size() >= 5 );


	double v_desired,v_desired_x,v_desired_y;
	if ( dest.type == Sdestination::Map_goal  )
	{
		v_desired_x = dest.x - current_point->x;
		v_desired_y = dest.y - current_point->y;
		v_desired = sqrt(v_desired_x*v_desired_x + v_desired_y*v_desired_y);
		v_desired_x *= desired_velocity_/v_desired;
		v_desired_y *= desired_velocity_/v_desired;
	}
	else //Sdestination::Stopping
	{
		v_desired_x = 0.0;
		v_desired_y = 0.0;
	}
	force_to_goal_ = Sforce(	social_forces_param->at(0) * (v_desired_x - current_point->vx),
			social_forces_param->at(0) * (v_desired_y - current_point->vy));
	return  force_to_goal_;
}


Sforce Cperson_abstract::force_sphe_prob( const SpointV_cov& interacting_person , const std::vector<double>* social_forces_param,
        const SpointV* virtual_current_point)
{
    //uniformly sampling around std
	double theta = interacting_person.angle_heading_point( this->current_pointV_ );
    Sforce f;
    Spoint p(interacting_person);
    for( unsigned int i=0; i< 4; ++i )
    {
        f += this->force_sphe( p-Spoint( sqrt(interacting_person.cov_xx())*cos(theta) ,  sqrt(interacting_person.cov_yy())*sin(theta)), social_forces_param, virtual_current_point );
        theta += PI/2.0;
    }
    return f*0.25;
}

Sforce Cperson_abstract::force_sphe_mahalanobis( const SpointV_cov& interacting_person , const std::vector<double>* social_forces_param,
        const SpointV* virtual_current_point)
{
	//TODO la calcula mal
    //Mahalanobis distance to interacting target: gives a relative distance that dpeneds on the covariance matrix
    double d_mah = sqrt( interacting_person.cov_dist( this->current_pointV_ ) );
    double theta = interacting_person.angle_heading_point( this->current_pointV_ );
    Spoint p(interacting_person),dt( d_mah*cos(theta) , d_mah*sin(theta) );
	return this->force_sphe( p-dt , social_forces_param, virtual_current_point );
}

Sforce Cperson_abstract::force_sphe_worst( const SpointV_cov& interacting_person , const std::vector<double>* social_forces_param,
        const SpointV* virtual_current_point)
{
    //worst case scenario: the interacting target is in the covariance elipsoid and the nearest point to center.
    double theta = interacting_person.angle_heading_point( this->current_pointV_ );
    Spoint p(this->current_pointV_),dt( interacting_person.cov_xx()*cos(theta) , interacting_person.cov_yy()*sin(theta) );
	return this->force_sphe( p-dt , social_forces_param, virtual_current_point );
}

Sforce Cperson_abstract::get_forces_person(Sforce&  force_to_goal, Sforce& force_int_person, Sforce& force_int_robot, Sforce& force_obstacle  ) const
{
	//make sure whenever you use this function to previously calculate the correspondent forces
	force_to_goal = force_to_goal_;
	force_int_person = force_int_person_;
	force_int_robot = force_int_robot_;
	force_obstacle = force_obstacle_;
	return force_to_goal_ + force_int_person_ + force_int_robot_+  force_obstacle_ ;
}

void Cperson_abstract::set_forces_person( Sforce force_to_goal, Sforce force_int_person, Sforce force_int_robot, Sforce force_obstacle )
{
	force_to_goal_ = force_to_goal;
	force_int_person_ = force_int_person;
	force_int_robot_ = force_int_robot;
	force_obstacle_ = force_obstacle;
}


Sbehavior::Sbehavior( unsigned int id ) : related_person_id(id)
{
	int N = Cperson_abstract::Count_behaviors;
	expectation.resize( N , 1.0/(double)N );
}

Sbehavior::~Sbehavior()
{

}
void Sbehavior::print() const
{
	std::cout << " Behavior to the related person  " << related_person_id <<
			" and expected behaviors = { " << expectation[0] <<
			" , " << expectation[1] << " , " << expectation[2] << " }" << std::endl;
	return;
}


Sforce force_sphe( const SpointV& center_person, const SpointV& interacting_person , const std::vector<double>* social_forces_param )
{
	//setting the corresponding SFM parameters:
	// if no forces param provided, exit program
	assert( social_forces_param != NULL );
	assert( social_forces_param->size() >= 5 );

	Sforce force;
	double f,d;
	//geometry calculations
	double dx = center_person.x - interacting_person.x;
	double dy = center_person.y - interacting_person.y;
	double vx = center_person.vx;
	double vy = center_person.vy;
	d = sqrt(dx*dx+dy*dy);
	dx /= d;
	dy /= d;
	//force module calculation
	f = social_forces_param->at(2)* exp((social_forces_param->at(4)-d) / social_forces_param->at(3));
	//force direction
	double phi = diffangle( atan2(vy,vx), atan2(-dy,-dx) );//minus difference vector
	double anisotropy = (social_forces_param->at(1) + (1-social_forces_param->at(1))*(1 + cos(phi))/2 );
	force.fx = f * dx * anisotropy;
	force.fy = f * dy * anisotropy;
	return force;
}
Sforce force_goal( const Sdestination dest, const SpointV& center_person , const std::vector<double>* social_forces_param )
{
	//setting the target type and its corresponding parameters:
	// if no forces param provided, exit program
	assert( social_forces_param != NULL );
	assert( social_forces_param->size() >= 5 );

	Sforce force_to_goal;
	double desired_velocity = center_person.v();


	double v_desired,v_desired_x,v_desired_y;
	if ( dest.type == Sdestination::Map_goal  )
	{
		v_desired_x = dest.x - center_person.x;
		v_desired_y = dest.y - center_person.y;
		v_desired = sqrt(v_desired_x*v_desired_x + v_desired_y*v_desired_y);
		v_desired_x *= desired_velocity/v_desired;
		v_desired_y *= desired_velocity/v_desired;
	}
	else //Sdestination::Stopping
	{
		v_desired_x = 0.0;
		v_desired_y = 0.0;
	}
	force_to_goal = Sforce(	social_forces_param->at(0) * (v_desired_x - center_person.vx),
			social_forces_param->at(0) * (v_desired_y - center_person.vy));
	return  force_to_goal;
}
