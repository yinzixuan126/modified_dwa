#include "prediction_behavior.h"
#include "scene_elements/person_behavior.h"
//#include <math.h>
#include <algorithm>
#include <stdio.h>
#include <Eigen/Dense>
#include <iostream>


Cprediction_behavior::Cprediction_behavior( double horizon_time , bool behavior_flag,
		Cperson_abstract::filtering_method filter, Cperson_abstract::update_person_method update_method ) :
			Cprediction_bhmip(filter,update_method),
	horizon_time_(horizon_time), behavior_flag_(behavior_flag)
{
	horizon_time_index_ =  (unsigned int) (horizon_time_/dt_) ;
	// paramters = {k,lambda,a,b,d} according to FerrerIcra2014
	force_params_balanced_ = {2.3, 0.56,3.13,2.91,0.2};
	force_params_aware_ = {2.3, 0.56,4.78,6.22,0.2};
	force_params_unaware_= {2.3, 0.56,0.98,0.16,0.2};

}

Cprediction_behavior::~Cprediction_behavior()
{
	//deletion of robot and persons is done on its parent class Cprediction_bhmip
}


Cperson_abstract* Cprediction_behavior::add_person_container( unsigned int id,
		std::list<Cperson_abstract*>::iterator iit )
{
	//this is the implementation of the abstract interface defined in Cscene_abstract, and although it
	// it has been defined in the father class, when calling 'update_scene()' the new container used will
	//be the one defined here
	Cperson_behavior* person = new Cperson_behavior(id,Cperson_abstract::Person, scene_force_type_, filtering_time_window_);

	person_list_.insert( iit , person );
	person->set_destinations(destinations_);

	return (Cperson_abstract*) person;
}


void Cprediction_behavior::scene_prediction()
{
	// Intentionality prediction
	Cprediction_bhmip::scene_intentionality_prediction_bhmip();

	if( behavior_flag_  )
	{
		scene_behavior_estimation();
	}

	//calculation of the trajectories
	scene_trajectory_prediction();

}

void Cprediction_behavior::scene_behavior_estimation()
{
	Sforce obs, goal;
	double dt;
	for( Cperson_abstract* observed_person: person_list_ )
	{
		//calculate the estimation for each force, according to the observed position and velocities
		dt = observed_person->get_diff_position().time_stamp;
		obs = Sforce( observed_person->get_diff_position().vx /dt , observed_person->get_diff_position().vy /dt);
		goal = observed_person->force_goal( observed_person->get_best_dest(), this->get_sfm_params( observed_person) );
		for ( Cperson_abstract* center_person : person_list_ )
		{
			if( *observed_person != *center_person  &&
					observed_person->get_current_pointV().distance( center_person->get_current_pointV() ) < 4.0 )//threshold of 4[m]
				this->behavior_update( observed_person, center_person, obs-goal );
		}
		//same calculation for the robot
		if ( robot_in_the_scene_ ){
		if( *observed_person != *robot_ &&
				observed_person->get_current_pointV().distance( robot_->get_current_pointV() ) < 4.0 )//threshold of 4[m]
			this->behavior_update( observed_person, robot_, obs-goal );
		}
	}

}


void Cprediction_behavior::behavior_update( Cperson_abstract* observed_person, Cperson_abstract* center_person, const Sforce& observed_int)
{
	//calculate the expected force
	double lh,force_mod, marginalization, norm(0.0);
	std::vector<double> posterior_prob( 3, 0.0 );
	Sforce expected_force;
	//find behavior for person observed, according to center person
	Sbehavior * behavior = center_person->find_behavior_estimation ( observed_person->get_id() );
	assert( behavior != NULL );
	for ( unsigned int i = 0; i< behavior->expectation.size() ; i++)
	{
		marginalization = 0.0;
		for( unsigned int j = 0; j< behavior->expectation.size() ; j++)
		{
			marginalization += behavior->expectation[j] * ( i == j ? 0.9 : 0.05 );//transition matrix parameters hrdcoded for 3 states
		}
		//HMM : the observed person recalculates its interaction force according to the sfm behaviors after interacting with center person,
		// which updates its behavior information upon these results
		expected_force = observed_person->force( center_person->get_current_pointV()  ,
				get_behavior_sfm_params( (Cperson_abstract::behavior_type)i  )  );
		force_mod = ( observed_int - expected_force ).module2();
		//std::cout << "force_mod " << force_mod << "for iteration " << i << std::endl;
		lh = exp( -0.5 * force_mod );//std = 1 TODO
		posterior_prob[i] = marginalization * lh;
		norm += posterior_prob[i];
	}

	//normalize HMM
	if ( norm <= 0 ) norm = 1e-20;
	for ( unsigned int i = 0; i< behavior->expectation.size() ; i++ )
		behavior->expectation[i] =  posterior_prob[i] / norm;

	//behavior->print();

}

const std::vector<double>*
Cprediction_behavior::get_behavior_sfm_params( Cperson_abstract::behavior_type best_behavior_type )
{
	switch( best_behavior_type)
	{
	  case Cperson_abstract::Balanced:
		return &force_params_balanced_;
	  case Cperson_abstract::Aware:
		return &force_params_aware_;
	  case Cperson_abstract::Unaware :
	  default :
		return &force_params_unaware_;
	}
}

const std::vector<double>*
Cprediction_behavior::get_sfm_int_params( const Cperson_abstract * center_person,	const Cperson_abstract * interacting_person )
{
	const std::vector<double>* result = Cscene_abstract::get_sfm_int_params(center_person, interacting_person);

	//if flag enabled, certain combinations of interactions may change, we modify only those that are different
	if ( behavior_flag_ )
	{
		result = get_behavior_sfm_params( center_person->get_best_behavior_to_person( interacting_person->get_id() ) );
	}
	return result;
}

void Cprediction_behavior::scene_trajectory_prediction(  )
{

	SpointV_cov virtual_next_pose;
	Sforce virtual_force_goal, virtual_force_int_person, virtual_force_obstacle, virtual_force_robot;
	for( Cperson_abstract* iit: person_list_ )
		iit->clear_prediction_trajectory();
	for( unsigned int t = 0;  t < horizon_time_index_; ++t)
	{
		for( Cperson_abstract* iit: person_list_ )
		{
			if ( iit->get_person_type() == Cperson_abstract::Person)
			{
				//1 time step propagation for all people
				virtual_force_goal = iit->force_goal(  iit->get_best_dest() , this->get_sfm_params(iit),
						&(iit->get_prediction_trajectory()->at(t)) );
				virtual_force_int_person = force_persons_int_virtual( iit, t );
				// force depending on map to be deprecated in real environments, but still used on simulations
				if( read_force_map_success_ )
					virtual_force_obstacle = get_force_map( iit->get_prediction_trajectory()->at(t).x,
							iit->get_prediction_trajectory()->at(t).y ) * 0.5;
				if(read_laser_obstacle_success_)
					virtual_force_obstacle = force_objects_laser_int_prediction( iit , t );
				//robot stays idle, no prediction is done
				if ( robot_in_the_scene_ )
				{
					virtual_force_robot =  iit->force( robot_->get_current_pointV() , this->get_sfm_int_params(iit,robot_),
						&(iit->get_prediction_trajectory()->at(t)) );
				}
				else
					virtual_force_robot = Sforce();
				iit->set_forces_person( virtual_force_goal, virtual_force_int_person, virtual_force_robot, virtual_force_obstacle );
				iit->prediction_propagation(dt_, iit->get_force_person() , t);
			}
			//else //robot case: no prediction is done since it is the objective of the planning class.
		}
	}

}


Sforce Cprediction_behavior::force_objects_laser_int_prediction( Cperson_abstract* person, unsigned int prediction_index)
{
	Sforce force_res;
	const SpointV_cov* virtual_current_point;
	if ( prediction_index == 0)
	{
		virtual_current_point = &(person->get_current_pointV());
	}
	else
	{
		virtual_current_point = &(person->get_prediction_trajectory()->at(prediction_index) );
	}
	for( Spoint iit : laser_obstacle_list_)
	{
		//there is a list for persons and another for robot(s). This function is for obstacles
		if( person->get_current_pointV().distance2( iit ) < 25.0)//square distance 5^2
		{
			force_res += person->force_sphe( iit , this->get_sfm_int_params(person) , virtual_current_point );
		}
	}
	return force_res;

}

Sforce Cprediction_behavior::force_persons_int_virtual(Cperson_abstract* center , unsigned long t)
{
	Sforce force_res;
	SpointV_cov person_int;
    for( auto iit : person_list_)
    {
		person_int = iit->get_prediction_trajectory( )->at(t);

		if(iit->get_person_type() == Cperson_abstract::Person &&
			*center != *iit && center->get_prediction_trajectory()->at(t)
			.distance( person_int ) < 15.0)
		{
		force_res += center->force( person_int ,  this->get_sfm_int_params(center,iit),
				&(center->get_prediction_trajectory()->at(t)) );
		//std::cout << "prediction, calculation of force using " << center->get_best_behavior_to_person( iit->get_id() ) << std::endl;
		}
    }
	return force_res;
}


void Cprediction_behavior::set_horizon_time( double t )
{
	horizon_time_ = t;
	horizon_time_index_ =  (unsigned int) (horizon_time_/dt_) ;
}

void Cprediction_behavior::clear_scene()
{

}


//calculates the current forces in the scene and updates each of the Cpersons and Crobot with
//instantaneous force information according to observations (current positions)
void Cprediction_behavior::calculate_current_forces( int pr_force_mode )
{
	SpointV_cov virtual_next_pose;
	Sforce virtual_force_goal, virtual_force_int_person, virtual_force_obstacle, virtual_force_robot;

	for( Cperson_abstract* iit: person_list_ )
	{
		if ( iit->get_person_type() == Cperson_abstract::Person)
		{
			//1 time step propagation for all people
			virtual_force_goal = iit->force_goal(  iit->get_best_dest() , this->get_sfm_params(iit) );
			virtual_force_int_person = Cscene_abstract::force_persons_int( iit);
			// force depending on map to be deprecated in real environments, but still used on simulations
			if( read_force_map_success_ )
				virtual_force_obstacle = get_force_map( iit->get_current_pointV().x,
						iit->get_current_pointV().y ) * 0.5;
			if(read_laser_obstacle_success_)
				virtual_force_obstacle = Cscene_abstract::force_objects_laser_int( iit  );
			//robot stays idle, no prediction is done
			if ( robot_in_the_scene_ )
			{
				switch( pr_force_mode )
				{
				case 0: //deterministic force, classical definition
					virtual_force_robot = iit->force( robot_->get_current_pointV() , this->get_sfm_int_params(iit,robot_), &(iit->get_current_pointV()) );
					break;
				case 1: //probabilistic force, sampling around the elipsoid
					virtual_force_robot = iit->force_sphe_prob( robot_->get_current_pointV(), get_sfm_int_params(iit,robot_),&(iit->get_current_pointV()) );
					break;
				case 2: //probabilistic force, mahalanobis distance
					virtual_force_robot = iit->force_sphe_mahalanobis( robot_->get_current_pointV(), get_sfm_int_params(iit,robot_), &(iit->get_current_pointV()));
					break;
				case 3: //probabilistic force, worst case scenario: covaraince ellipsoid nearer to the center point
					virtual_force_robot = iit->force_sphe_worst( robot_->get_current_pointV(), get_sfm_int_params(iit,robot_),&(iit->get_current_pointV()));
					break;
				}
			}
			else
				virtual_force_robot = Sforce();
			iit->set_forces_person( virtual_force_goal, virtual_force_int_person, virtual_force_robot, virtual_force_obstacle );
		}
	}
	//robot update, if any
	if ( robot_in_the_scene_ )
	{
		virtual_force_goal = robot_->force_goal(  robot_->get_best_dest() , this->get_sfm_params(robot_) );
		virtual_force_int_person = Cscene_abstract::force_persons_int( robot_ );
		// force depending on map to be deprecated in real environments, but still used on simulations
		if( read_force_map_success_ )
			virtual_force_obstacle = get_force_map( robot_->get_current_pointV().x,
					robot_->get_current_pointV().y ) * 0.5;
		if(read_laser_obstacle_success_)
			virtual_force_obstacle = Cscene_abstract::force_objects_laser_int( robot_  );
		virtual_force_robot = Sforce();
		robot_->set_forces_person( virtual_force_goal, virtual_force_int_person, virtual_force_robot, virtual_force_obstacle );
	}
}
