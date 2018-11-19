#include "scene_abstract.h"
#include <stdio.h>
#include <iostream>
#include <math.h>


Cscene_abstract::Cscene_abstract( Cperson_abstract::filtering_method filter,
		Cperson_abstract::update_person_method update_method,
		Cperson_abstract::force_type type):
robot_(NULL), dt_(0.1) , now_(0.0) , read_laser_obstacle_success_(false), read_force_map_success_(false), read_destination_map_success_(false),
scene_force_type_(type), filtering_method_(filter), update_method_(update_method),
robot_in_the_scene_(false)
{

	switch( scene_force_type_ )
	{
	case Cperson_abstract::Collision_Prediction:
		// Zanlungo collision prediction parameters , {k,lambda,A,B,d}
		social_forces_param_to_person_ = {1.52, 0.29,1.13,0.71,0.0};
		//Person-Robot Spherical parameters obtained using our optimization method
		social_forces_param_to_robot_ = {1.52, 0.29,1.13,0.71,0.0};
		//set_social_force_parameters_person_robot( force_params_to_vector(2.3, 0.59,2.66,0.79,0.4) );
		//Obstacle spherical parameters obtained using our optimization method
		social_forces_param_to_obs_ = {2.3, 1.0,10.0,0.1,0.2};
		break;
	case Cperson_abstract::Elliptical:
		//Default Zanlungo Elliptical parameters
		social_forces_param_to_person_ = {1.19, 0.08,1.33,0.34,1.78};
		//Person-Robot Spherical parameters obtained using our optimization method
		social_forces_param_to_robot_ = {2.3, 0.59,2.66,0.79,0.4};
		//Obstacle spherical parameters obtained using our optimization method
		social_forces_param_to_obs_ = {2.3, 1.0,10.0,0.1,0.2};
		break;
	case Cperson_abstract::Spherical:
		//TODO set as default the paramters calculated in ICRA'2104 (Balanced behavior)
		//Default Zanlungo Spherical parameters (2.3, 0.08,1.33,0.64,0.16)
		social_forces_param_to_person_ = {4.9, 1.0,10.0,0.64,0.16};//B=0.34, changed to 0.64
		//Person-Robot Spherical parameters obtained using our optimization method
		social_forces_param_to_robot_ = {2.3, 0.59,2.66,0.79,0.4};
		//Obstacle spherical parameters obtained using our optimization method
		social_forces_param_to_obs_ = {2.3, 1.0,10.0,0.6,0.2};
		break;
	}



}

Cscene_abstract::~Cscene_abstract()
{

}

void Cscene_abstract::set_destinations( std::vector<Sdestination>& dest )
{
	destinations_ = dest;
	for( std::list<Cperson_abstract*>::iterator iit = person_list_.begin() ; iit != person_list_.end() ; iit++ )
	{
		(*iit)->set_destinations( dest );
	}
	// robot destinations are not required, are set if planned
}

Cperson_abstract* Cscene_abstract::add_person( unsigned int id )
{
	std::list<Cperson_abstract*>::iterator iit = person_list_.begin();
	if ( !person_list_.empty())
	{
		while( iit != person_list_.end())
		{
			if ( (*iit)->get_id()  > id ) //interrupts if the id is greater for ordering the list
			{
				break;
			}
			iit++;
		}
	}

	return add_person_container(id,iit);//virtual function adding the specific person container
}

void Cscene_abstract::remove_person( unsigned int id )
{
	std::list<Cperson_abstract*>::iterator iit;
	Cperson_abstract* person;
	if (find_person( id,  &person, iit ))
	{
		delete person;
		person_list_.erase(  iit );
	}
}

void Cscene_abstract::update_robot(Spose observation)
{
	now_ = observation.time_stamp;
	if ( robot_in_the_scene_ )
	{
		robot_->add_pose( observation );
	}
	else
	{
		robot_in_the_scene_ = true;
		robot_ = new Crobot(0,Crobot::Differential,scene_force_type_);
		robot_->add_pose( observation );
	}
}

bool Cscene_abstract::find_person(unsigned int id)
{
	std::list<Cperson_abstract*>::iterator iit = person_list_.begin();
	if ( !person_list_.empty())
	{
		while( iit != person_list_.end())
		{
			// only returns persons (robot wont be returned if sought)
			if( (*iit)->get_id()  == id && (*iit)->get_person_type() == Cperson_abstract::Person )
			{
				return true;
			}
			if ( (*iit)->get_id()  > id ) //interrupts if the id is greater for ordering the list
			{
				break;
			}
			iit++;
		}
	}
	return false;
}

bool Cscene_abstract::find_person(unsigned int id , Cperson_abstract** person)
{
	std::list<Cperson_abstract*>::iterator iit = person_list_.begin();
	if ( !person_list_.empty())
	{
		while( iit != person_list_.end())
		{
			if( (*iit)->get_id()  == id && (*iit)->get_person_type() == Cperson_abstract::Person)
			{
				*person = *iit;
				return true;
			}
			if ( (*iit)->get_id()  > id ) //interrupts if the id is greater for ordering the list
			{
				break;
			}
			iit++;
		}
	}
	return false;
}

bool Cscene_abstract::find_person(unsigned int id , Cperson_abstract** person,
		std::list<Cperson_abstract*>::iterator& it)
{
	std::list<Cperson_abstract*>::iterator iit = person_list_.begin();
	if ( !person_list_.empty())
	{
		while( iit != person_list_.end())
		{
			if( (*iit)->get_id()  == id && (*iit)->get_person_type() == Cperson_abstract::Person)
			{
				*person = *iit;
				it = iit;
				return true;
			}
			if ( (*iit)->get_id()  > id ) //interrupts if the id is greater for ordering the list
			{
				break;
			}
			iit++;
		}
	}
	return false;
}

void Cscene_abstract::print()
{
	for( std::list<Cperson_abstract*>::iterator iit = person_list_.begin() ; iit != person_list_.end() ; iit++ )
	{
		(*iit)->print_dest();
		(*iit)->print();
	}
}

void Cscene_abstract::read_laser_scan( const std::vector<Spoint>&  laser_scan )
{
	//TODO take into account both back and front laser scans
	laser_obstacle_list_.clear();
	for ( Spoint p : laser_scan )
	{
		//filter laser points corresponding to persons and obstacles
		if ( !point_corresponds_person(p) && !point_belongs_to_laser_obstacle(p) )
		{
			//new cluster scale-invariant laser points is created
			laser_obstacle_list_.push_back( p );
		}
	}
	read_laser_obstacle_success_ = true;//flag to indicate that indeed we can make use of laser information regarding obstacles.
}

bool Cscene_abstract::point_corresponds_person( Spoint laser_point )
{
	for( Cperson_abstract* iit : person_list_ )
	{
		if( laser_point.distance2( iit->get_current_pointV() ) < 1.0 )//sqrt(1) = 1[m] is considered person
			return true;
	}
	return false;
}

bool Cscene_abstract::point_belongs_to_laser_obstacle( Spoint p)
{
	double dist_thr,d2;
	d2 = p.distance2( robot_->get_current_pointV()  );//distance to robot
	if( d2 < 1.0  )//calculates basic distance to robot, near distance, the thr is lower
	{
		dist_thr = 0.09;//sqrt(0.09) = 0.3[m] is considered obstacle
	}
	else if( d2 < 16.0 )//4 [m]
	{
		dist_thr = 0.2;//sqrt(0.16) = 0.45[m] is considered obstacle
	}
	else
	{
		dist_thr = 0.36;//sqrt(0.09) = 0.6[m] is considered obstacle
	}
	for( Spoint o : laser_obstacle_list_ )
	{

		if ( o.distance2( p )  <  dist_thr )
			return true;
	}
	return false;
}

Sforce Cscene_abstract::force_objects_laser_int( Cperson_abstract* person)
{
	Sforce force_res;
	for( Spoint iit : laser_obstacle_list_)
	{
		//there is a list for persons and another for robot(s). This function is for obstacles
		if( person->get_current_pointV().distance2( iit ) < 25.0)//square distance 5^2
		{
			force_res += person->force_sphe( iit , this->get_sfm_int_params( person ) );
		}
	}
	return force_res;

}

Sforce Cscene_abstract::force_persons_int( Cperson_abstract* center )
{
	Sforce force_res;
	for( std::list<Cperson_abstract*>::iterator iit = person_list_.begin() ; iit != person_list_.end() ; iit++ )
	{
		//there is a list for persons and another for robot(s). This function is for persons
		if( *center != *(*iit) && center->get_current_pointV().distance( (*iit)->get_current_pointV() ) < 15.0)
		{
			force_res += center->force( (*iit)->get_current_pointV() ,  this->get_sfm_int_params( center , *iit) );
		}
	}
	return force_res;

}

const std::vector<double>* Cscene_abstract::get_sfm_params( const Cperson_abstract * center_person)
{
	assert( center_person != NULL );
	switch( center_person->get_person_type() )
	{
	case Cperson_abstract::Person :
		return &social_forces_param_to_person_;
	case Cperson_abstract::Robot :
		return &social_forces_param_to_robot_;
	case Cperson_abstract::Obstacle :
	default:
		return &social_forces_param_to_obs_;
	}
}

const std::vector<double>* Cscene_abstract::get_sfm_int_params( const Cperson_abstract * center_person, const Cperson_abstract * interacting_person)
{
	assert( center_person != NULL );
	Cperson_abstract::target_type interacting_type;
	if ( interacting_person == NULL )
	{
		interacting_type = Cperson_abstract::Obstacle;
	}
	else
	{
		interacting_type = interacting_person->get_person_type();
	}

	//return the corresponding SFM parameters
	if( center_person->get_person_type() == Cperson_abstract::Person
			&& interacting_type != Cperson_abstract::Obstacle )
		//	&& interacting_type == Cperson_abstract::Person ) //TODO cambio para ver si funciona la relacion asimetrica
	{
		return &social_forces_param_to_person_;
	}
	else if( center_person->get_person_type() == Cperson_abstract::Robot &&
			interacting_type != Cperson_abstract::Obstacle)
	{
		return &social_forces_param_to_robot_;
	}
	else
	{
		return &social_forces_param_to_obs_;
	}

	return NULL;
}

bool Cscene_abstract::read_force_map( const char * path )
{
	FILE * fid;
	float fx,fy;
	int obstacle;
	fid = fopen( path , "r");
	if (fid==NULL ) return read_force_map_success_;//false at ini
	//fscanf(fid,"%f",&fx);
	int fs =fscanf(fid,"%f",&min_x_);
	fs = fscanf(fid,"\n%d",&map_number_x_);
	fs =fscanf(fid,"\n%f",&min_y_);
	fs =fscanf(fid,"\n%d",&map_number_y_);
	fs = fscanf(fid,"\n%f",&map_resolution_);
	max_x_ = min_x_+  map_number_x_ * map_resolution_;
	max_y_ = min_y_+  map_number_y_ * map_resolution_;

	if( map_number_x_ * map_number_y_ <= 0 ) return read_force_map_success_;//false empty
	std::cout << "number of elements (x,y ) = " << map_number_x_ << " , " << map_number_y_ << std::endl;
	force_map_.reserve( map_number_x_ * map_number_y_ );
	obstacle_map_.reserve( map_number_x_ * map_number_y_ );
	for( unsigned int i = 0; i< map_number_y_*map_number_x_; ++i)
	{
		//fscanf(fid,"\n%f %f %d",&fx,&fy,&obstacle);
		fs = fscanf(fid,"\n%f",&fx);
		fs = fscanf(fid," %f",&fy);
		fs = fscanf(fid," %d",&obstacle);
		force_map_.push_back( Sforce(fx,fy) );
		obstacle_map_.push_back( (bool)obstacle );
	}
	fclose(fid);
	fs &= 0;//to avoid warning
	//cout << "map_number_x_" << map_number_x_ << " map_number_rows = " << map_number_y_ <<
	//		"  and the vector map has a total of elements = " << force_map_.size() << endl;
	read_force_map_success_ = true;
	return read_force_map_success_;
}

Sforce Cscene_abstract::get_force_map( double x, double y )
{
	if ( read_force_map_success_ && x >= min_x_ && x <= max_x_ &&  y >= min_y_ && y <= max_y_ )
	{
		//by construction in the .m, the vector is read in columns
		// iit = floor( (x-vv(1)) / vv(5))   + floor((y-vv(3)) / vv(5))*M +1;
		//cout << "iteration" << int( (x-min_x_) / map_resolution_) * map_number_y_  + int ((y-min_y_) / map_resolution_) << endl;
		return force_map_[ int( (x-min_x_) / map_resolution_) + int ((y-min_y_) / map_resolution_) * map_number_x_ ];
	}
	else
		return Sforce();
}

bool Cscene_abstract::is_cell_clear_map( double x, double y )
{
	//returns true if cell is clear (or outside the map) and false if there is an obstacle
	if ( read_force_map_success_ && x >= min_x_ && x <= max_x_ &&  y >= min_y_ && y <= max_y_ )
	{
		//by construction in the .m, the vector is read in columns
		// iit = floor( (x-vv(1)) / vv(5))   + floor((y-vv(3)) / vv(5))*M +1;
		//cout << "iteration" << int( (x-min_x_) / map_resolution_) * map_number_y_  + int ((y-min_y_) / map_resolution_) << endl;
		return obstacle_map_[ int ( (x-min_x_) / map_resolution_) + int ((y-min_y_) / map_resolution_) * map_number_x_ ];
	}
	else
		return true;
}

void Cscene_abstract::get_map_params(float &min_x, float &max_x, float &min_y, float &max_y, float &resolution,
		unsigned int &map_number_x, unsigned int &map_number_y)
{
	min_x = min_x_;
	max_x = max_x_;
	min_y = min_y_;
	max_y = max_y_;
	resolution = map_resolution_;
	map_number_x = map_number_x_;
	map_number_y = map_number_y_;
}

unsigned int Cscene_abstract::xy_to_m(double x, double y)
{
	if ( read_force_map_success_ && x >= min_x_ && x <= max_x_ &&  y >= min_y_ && y <= max_y_ )
		{
			return int ( (x-min_x_) / map_resolution_) + int ((y-min_y_) / map_resolution_) * map_number_x_ ;
		}
	return 0;
}
Spoint Cscene_abstract::m_to_xy( unsigned int m)
{
	double x = min_x_+(m - m/map_number_x_ * map_number_x_ ) * map_resolution_;
	double y = min_y_ + m/map_number_x_*map_resolution_;
	return Spoint( x,y );
}

bool Cscene_abstract::read_destination_map( const char * path )
{
	/* * how to use this function: you need a destination's file
	number of destinations in the scene
	dest_id_1 , x , y , pr, n_neighbours , neighbour1, neighbour2, ...
	dest_id_2 , x , y , pr, n_neighbours , neighbour1, neighbour2, ...
	2
	1 0.0 0.0 0.5 1 2
	2 10.0 10.0 0.5 1 1
	*/
	FILE * fid;
	fid = fopen( path , "r");
	if (fid==NULL) return read_destination_map_success_;//false at ini
	//fscanf(fid,"%f",&fx);
	int n_dest;
	int fs = fscanf(fid,"%d",&n_dest);
	destinations_.clear();
	destinations_.reserve( n_dest );
	int id, nn, n;
	float x,y,pr;
	std::vector<int> neigh;
	for( unsigned int i = 0; i< (unsigned) n_dest; ++i)
	{
		//dest_ id , x , y , pr, n_neighbours , neighbour1, neighbour2, ...
		fs = fscanf(fid,"\n%d, %f, %f, %f, %d",&id,&x,&y,&pr,&nn);
		neigh.clear();
		neigh.reserve(nn);
		for(unsigned int j = 0; j < (unsigned) nn; ++j)
		{
			fs = fscanf(fid,", %d" , &n);
			neigh.push_back(n);
		}

		destinations_.push_back( Sdestination(id,x,y,pr, Sdestination::Map_goal , neigh) );
		destinations_.back().print();
	}
	fclose(fid);
	fs *= 0;
	//check that there are destinations
	if( destinations_.empty() ) return read_destination_map_success_;

	read_destination_map_success_ = true;
	//update robot and people destinations
	this->set_destinations( destinations_ );
	return read_destination_map_success_;
}

