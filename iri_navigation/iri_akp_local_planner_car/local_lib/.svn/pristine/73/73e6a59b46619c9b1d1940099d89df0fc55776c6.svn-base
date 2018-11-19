/*
 * plan_local_nav.cpp
 *
 *  Created on: Dec 22, 2013
 *      Author: gferrer
 */

#include "nav/plan_local_nav.h"
#include <algorithm>
#include <iostream>


Cplan_local_nav::Cplan_local_nav(double horizon_time, unsigned int max_iter,
		plan_mode mode) :
Cprediction_behavior(horizon_time),
plan_mode_(mode), distance_mode_(Cplan_local_nav::Euclidean), global_mode_(Cplan_local_nav::Scalarization),
max_iter_(max_iter), workspace_radii_(0.0), std_goal_workspace_(1.0), max_v_by_system_(0.8), cost_angular_(0.25),
xy_2_goal_tolerance_(0.04), v_goal_tolerance_(0.05), distance_to_stop_(1.0),
alpha_(1.0), gamma_(1.0), delta_(1.0),
last_pose_command_(Spose()),
std_cost_int_forces_(1.0),
std_cost_robot_(1.0),
std_cost_obstacles_(1.0),
std_cost_past_traj_(1.0),
std_cost_distance_(1.0),
std_cost_orientation_(1.0),
gaussian_constant_( 2.0/(2.0*PI) ),
ppl_collision_mode_(0),
pr_force_mode_(0)
{
    edge_.reserve((size_t)max_iter_);
    cost_int_forces_.reserve((size_t)max_iter_);
    cost_robot_.reserve((size_t)max_iter_);
    cost_obstacles_.reserve((size_t)max_iter_);
    //cost_local_minima_.reserve((size_t)max_iter_);
    cost_distance_.reserve((size_t)max_iter_);
    cost_orientation_.reserve((size_t)max_iter_);
    cost_past_traj_.reserve((size_t)max_iter_);
    nodes_in_branch_.reserve((size_t)max_iter_);
	random_goals_.reserve( (size_t)max_iter_ );

    //Initialization of robot: there is always a robot if planning
	robot_in_the_scene_ = true;
	robot_ = new Crobot(0,Crobot::Differential,scene_force_type_);
    robot_->reserve_planning_trajectory(max_iter_);

    //random generator
    generator_.seed();

    //planner cost paramters
    cost_parameters_.reserve(7);
    cost_parameters_.push_back(1.0);// [0] Goal cost
    cost_parameters_.push_back(1.0);// [1] orientation cost
    cost_parameters_.push_back(1.0);// [2] Robot cost
    cost_parameters_.push_back(1.0);// [3]Interacting people cost
    cost_parameters_.push_back(0.25);// [4] potential time
    cost_parameters_.push_back(1.0);// [5] obstacles cost
    cost_parameters_.push_back(1.0);// [6] past trajectory function cost
    cost_parameters_.push_back(1.0);// [7] local minima scape cost

    // all ppl time
    filtering_time_window_ = 1.0;


}

Cplan_local_nav::~Cplan_local_nav()
{
	//all memory allocations are freed in Cprediction_behavior/bhmip
}

bool Cplan_local_nav::robot_plan(Spose& pose_command, double dt)
{
	this->Cprediction_behavior::scene_prediction();
	//kinodynamic rrt
	bool result;
	if ( robot_plan_anticipative_krrt() )
	{
		this->Cprediction_behavior::calculate_current_forces( pr_force_mode_ );//for plotting purposes
		last_pose_command_ = get_best_planned_pose( dt );
		//last_pose_command_.print();
		//robot_->print();
		result = true;
	}
	else
	{
		last_pose_command_ = Spose();
		result = false;
	}
	pose_command = last_pose_command_;
	return result;
}

void Cplan_local_nav::set_robot_goal( const Spoint& goal )
{
	goal_ = goal;
	Sdestination dest_goal(0,goal.x,goal.y,1.0);
	std::vector<Sdestination> robot_dest(1,dest_goal);
	robot_->set_destinations( robot_dest );
	robot_->set_best_dest( dest_goal );
}

bool Cplan_local_nav::init_robot_plan()
{
	// clear all data structures and push back the initial state : current state
    edge_.clear();
    cost_int_forces_.clear();
    cost_robot_.clear();
    cost_obstacles_.clear();
    //cost_local_minima_.clear();
    cost_distance_.clear();
    cost_orientation_.clear();
    nodes_in_branch_.clear();
    random_goals_.clear();
    edge_.push_back( Sedge_tree(0) );
    cost_int_forces_.push_back( 0.0 );
    cost_robot_.push_back(0.0);
    cost_obstacles_.push_back( 0.0 );
    //cost_local_minima_.push_back(0.0);
    cost_distance_.push_back( 0.0 );
    cost_orientation_.push_back( 0.0 );
    cost_past_traj_.resize( max_iter_ , 0.0 );//no clear is needed, only the i-th iteration end of branch will contain a value
    nodes_in_branch_.push_back(1.0);

    //Correct current robot location and state: hypothesis the current robot state and the past robot action may differ.
    //So we must take into account the delay and the system platform control, specially for the w, and propagate an estimation
    //of the state at time t_now to time t_now + t_processing and sending command
    robot_->correct_state_to_delay( last_pose_command_,  now_ , dt_ );
    robot_->clear_planning_trajectory();//first pose is inserted in the trajectory vector

    // ----------------------------------------------------------------------
    // set navigation goals
    workspace_radii_ = horizon_time_ * robot_->get_v_max(); //approximation of the max distance
	//if ( workspace_radii_ < 0.1 ) return false;
    local_v_goal_tolerance_ = robot_->get_v_max()+0.1; //sets the tolerance to the max value to always return true
	reaching_goal_ = false;
	if (workspace_radii_ > robot_->get_current_pointV().distance( goal_) )
	{
		local_goal_ = goal_;
		// New mode entering subgoals, the radius should be the maximum
		//workspace_radii_ = robot_->get_current_pointV().distance( goal_) + 0.5;
		//Set velocity to tolerance when reaching the final goal, only very near
		if( robot_->get_current_pointV().distance2( goal_) < xy_2_goal_tolerance_*2.0 )
		{
			//the velocity tolerance depends on the parameter, and the algorithm will check this condition in order to consider
			//a goal properly reached
			local_v_goal_tolerance_ = v_goal_tolerance_;
			reaching_goal_ = true;
		}

	}
	else
	{
		// In this case the final velocity is not an issue because the plan will never reach it final goal
		Spoint diff = goal_ - (Spoint)robot_->get_current_pointV();
		double r = sqrt(diff.x*diff.x +  diff.y * diff.y);
		local_goal_ = Spoint( robot_->get_current_pointV().x + workspace_radii_ * diff.x / r ,
				robot_->get_current_pointV().y + workspace_radii_ * diff.y / r );
	}
    robot_->set_rnd_local_goal( local_goal_ );
    //now_ = set when updated

    // ----------------------------------------------------------------------
    //select persons considered in the scene and reserve memory for planning
    Spoint robot_position = (Spoint) robot_->get_current_pointV();
    double d_ini,d_end,radii_2(workspace_radii_*workspace_radii_),d_min(1e10);
    nearby_person_list_.clear();
    for( auto iit: person_list_ )
    {
    	//It requires prior trajectory prediction, careful...
    	d_ini = iit->get_prediction_trajectory()->front().distance2( robot_position );
    	d_end = iit->get_prediction_trajectory()->back().distance2( robot_position );
    	if( d_ini < radii_2 || d_end < radii_2 )
    	{
    		nearby_person_list_.push_back( iit );
    		iit->clear_planning_trajectory();
    		iit->reserve_planning_trajectory( max_iter_ );//if already of this size, does nothing
    	}

    	//check for the neares obstacle in roder to calculate velocities
    	if ( d_ini < d_min)
    		d_min = d_ini;
    }

    // ----------------------------------------------------------------------
    //number of nearby obstacles: first approach, just count them...
    int number_of_obstacles(0);
    for( auto iit: laser_obstacle_list_ )
    {
    	d_ini = iit.distance2( robot_position );
    	if( d_ini < radii_2  )
    	{
    		number_of_obstacles++;
    	}
    	if ( d_ini < d_min )
    		d_min = d_ini;
    }

    // calculate the desired robot velocity depending on the nearbiest obstacle/person and goal
	double distance_to_goal = robot_->get_current_pointV().distance( goal_);
	// when the goal, starts to stop, higher priority than ppl
    if( distance_to_goal < distance_to_stop_ )
		robot_->set_v_max( max_v_by_system_ * distance_to_goal / distance_to_stop_  );//distance_to_stop is never 0
    // if not near the goal, normal velocity regulation according to nearby ppl
    else if ( d_min < 1.0 )
    	robot_->set_v_max( max_v_by_system_ * 0.75 );
    else if ( d_min < 3.0)
    	robot_->set_v_max( max_v_by_system_ * 0.85);
    else if ( d_min < 5.0 )
    	robot_->set_v_max( max_v_by_system_ * 0.95 );
    else
    	robot_->set_v_max( max_v_by_system_  );
    //depending on the number of persons considered, set the std of the std_goal_workspace [rad]
    switch( (int)nearby_person_list_.size() + number_of_obstacles)
    {
      case 0:
		std_goal_workspace_ = 0.4;
		break;
      case 1:
		std_goal_workspace_ = 1.0;
		break;
      case 2:
		std_goal_workspace_ = 1.4;
		break;
      case 3:
		std_goal_workspace_ = 1.6;
		break;
      case 4:
		std_goal_workspace_ = 1.8;
		break;
      case 5:
		std_goal_workspace_ = 2.0;
		break;
      default:
      case 6:
  		std_goal_workspace_ = 2.1;
  		break;
    }


    return true;
}

bool Cplan_local_nav::robot_plan_anticipative_krrt()
{
	//1- Initialization
	init_robot_plan();
	Sedge_tree input(0);
	Spoint random_goal;
	unsigned int parent_vertex_index(0),previous_parent_vertex(0), number_of_cost_to_go(0);
	//first goal is the main goal
	random_goal = local_goal_;
	random_goals_.push_back( random_goal );
	//bool collision_detected = false;
	collision_detected_.resize(max_iter_,false);
	collision_detected_[0] = false;
	double distance2_to_goal(1.0);

	for( unsigned int i = 1; i<max_iter_ && number_of_cost_to_go < max_iter_/5; ++i )
	{
		distance2_to_goal = robot_->get_robot_planning_trajectory()->back().distance2( local_goal_ );
		//std::cout << "current diff time "  << robot_->get_robot_planning_trajectory()->back().time_stamp - now_ << std::endl;
		if( robot_->get_robot_planning_trajectory()->back().time_stamp - now_ > horizon_time_ ||
				collision_detected_[i-1]  ||
				(distance2_to_goal  < xy_2_goal_tolerance_ && //distance to goal and velocity
						robot_->get_robot_planning_trajectory()->back().v < local_v_goal_tolerance_ )) //this local tolerance depends if the goal is reachable before h
		{


			// add end of branch, either horizon time is reached or goal is reached
			if (!collision_detected_[i-1] )
			{
				end_of_branches_index_.push_back( i-1 );
				//distance to last trajectory calculated: only last calculated position in both paths... simplified version
				if ( best_planning_trajectory_.empty() )
					cost_past_traj_[i-1] = 0.0;
				else
					cost_past_traj_[i-1] = best_planning_trajectory_.front().distance2( robot_->get_robot_planning_trajectory()->back()  );
			}
			//colisions are discarted
			else {
				//std::cout << " collision detected" << std::endl;
			}
		//2- Sample workspace
			random_goal = sample_workspace();
			random_goals_.push_back( random_goal );
		    robot_->set_rnd_local_goal( random_goal );
		//3- find nearest vertex to expand the tree
			parent_vertex_index = find_nearest_vertex( random_goal );
			++number_of_cost_to_go;
			previous_parent_vertex = i;
			reset_scene_persons_propagation_flag();
		}
		else
		{
			parent_vertex_index = previous_parent_vertex;
			previous_parent_vertex = i;
		}

		//check to see if goal is currently the local goal; in that case, the velocity has to be adjusted to stop
		if ( reaching_goal_ )
		{
			//set robot desired velocity to 0 when inside the goal ball
			if( distance2_to_goal < xy_2_goal_tolerance_ )
			{
				robot_->set_desired_velocty( 0.0 );
			}
			else // robot desired velocity is again max velocity
				robot_->set_desired_velocty( robot_->get_v_max () );//robot always tries to be at max velocity, modified by nearby ppl
		}
		//4-calculate the input u to propagate towards the random goal
		input = calculate_edge(parent_vertex_index, Sdestination(0,random_goal.x,random_goal.y) );
		//5- Propagate tree.
		collision_detected_[i] = propagate_vertex( parent_vertex_index, input );
		calculate_cost(parent_vertex_index, input);

		//6-calculate the propagation cost
		edge_.push_back( input );
	}
	//7- return tree. Seek for the minimum cost branch of the tree


	unsigned int min_branch_index = global_min_cost_index( );

	// cout best candidate paths and costs
	//std::cout << "best path candidate = " << min_branch_index << std::endl;
	//std::cout  << 	cost_int_forces_[min_branch_index] << " , " << cost_robot_[min_branch_index] << " , " <<  cost_obstacles_[min_branch_index] << " , " <<
	//		cost_distance_[min_branch_index] << " , " << cost_orientation_[min_branch_index] << std::endl;


	best_plan_vertex_index_.clear();
	best_planning_trajectory_.clear();
	if ( end_of_branches_index_.empty() ) return false;
	//8- calculate vector of vertex. In reverse order the vertexes of the best path (initial not
	// included as it doesn't provide information)
	//std::cout << "% distance, orientation, robot, int, obstacles" << std::endl;
	do
	{
		best_plan_vertex_index_.push_back( min_branch_index);
		best_planning_trajectory_.push_back( robot_->get_robot_planning_trajectory()->at(min_branch_index) );
		min_branch_index = edge_[ min_branch_index ].parent;
	}while( min_branch_index > 0 );


	// clear data structures. Out of this function, there is no guarantee that data has not changed
    nearby_person_list_.clear();
    //end of trajectories indexes
    end_of_branches_index_.clear();
    return true;
}

Spoint Cplan_local_nav::sample_workspace(  )
{
	Spoint sample;
	double theta(0.0);
	std::uniform_real_distribution<double> sample_x( robot_->get_current_pose().x - workspace_radii_,
			robot_->get_current_pose().x + workspace_radii_ );
	std::uniform_real_distribution<double> sample_y( robot_->get_current_pose().y - workspace_radii_,
			robot_->get_current_pose().y + workspace_radii_ );
	double dx = goal_.x - robot_->get_current_pose().x;
	double dy = goal_.y - robot_->get_current_pose().y;
	std::normal_distribution<double> sample_g( atan2(dy,dx), std_goal_workspace_ );
	switch(plan_mode_)
	{
	  case F_RRT_Uniform :
		sample = Spoint( sample_x(generator_), sample_y(generator_)  );
		break;
	  case F_RRT_Gauss_Circle :
	  case F_RRT_GC_alpha :
	  default:
		theta = sample_g(generator_);
		sample = Spoint( robot_->get_current_pose().x + workspace_radii_*cos(theta),
				robot_->get_current_pose().y + workspace_radii_*sin(theta));
		break;
	}

	//robot parameters sampling if required
	//set the random alpha variable, if necessary
	if( plan_mode_ == F_RRT_GC_alpha )
	{

		std::uniform_int_distribution<int> sample_behavior( 0 , 2 );
		double epsilon(0.4);
		switch( sample_behavior(generator_) )
		{
		case 0://robot unaware
			alpha_ = 1.0+epsilon;gamma_ = 1.0-epsilon;delta_ = 1.0-epsilon;
			break;
		case 1://robot aware
			alpha_ = 1.0-epsilon;gamma_ = 1.0+epsilon;delta_ = 1.0+epsilon;
			break;
		case 2://robot balanced
		default:
			alpha_ = 1.0;gamma_ = 1.0;delta_ = 1.0;
			break;
		}

		//std::cout << alpha_ << " , " << gamma_ << " , " << delta_ << std::endl;
	}


	return sample;
}

void Cplan_local_nav::reset_scene_persons_propagation_flag()
{
	for( Cperson_abstract* iit : nearby_person_list_  )
	{
		iit->reset_propagation_flag();
	}
}

unsigned int Cplan_local_nav::find_nearest_vertex( const Spoint& random_goal )
{
	double min_dist(1e10), result;
	unsigned int nearest_vertex_index = 0;
	for( unsigned int i = 0 ; i < robot_->get_robot_planning_trajectory()->size() ; i+=2 )
	{

		if ( !collision_detected_[i] )
		{
			result = cost_to_go(random_goal,i);
			if ( result  < min_dist  )
			{
				min_dist = result;
				nearest_vertex_index = i;
			}
		}

	}
	if ( robot_->get_robot_planning_trajectory()->at( nearest_vertex_index ).time_stamp > horizon_time_ + now_ )
		return 0;
	return nearest_vertex_index;
}

Sedge_tree Cplan_local_nav::calculate_edge( unsigned int parent_vertex, const Sdestination& random_goal)
{
	Sforce f_goal = robot_->force_goal( random_goal, get_sfm_params(robot_),&(robot_->get_planning_trajectory()->at(parent_vertex))  );
	Sforce f_int = force_persons_int_planning_virtual( robot_, parent_vertex );

    Sforce f_obs;
    //map force, used for simulations
	if( read_force_map_success_ )
		f_obs = get_force_map(robot_->get_robot_planning_trajectory()->at(parent_vertex).x,
    		robot_->get_robot_planning_trajectory()->at(parent_vertex).y);
    //obstacles due to laser scans. for real environments has priority over map forces
	if(read_laser_obstacle_success_)
		f_obs = force_objects_laser_int_planning_virtual( robot_, parent_vertex, 25.0, true );
	return Sedge_tree( parent_vertex, f_goal*alpha_ + f_int*gamma_ + f_obs*delta_, f_goal, f_int, f_obs );
}

bool Cplan_local_nav::propagate_vertex( unsigned int parent_index , const Sedge_tree& u)
{
	// Robot propagation and cost ----------------------------------------------------------------------
	unsigned int index_to_copy;
	SpointV_cov virtual_next_pose;
	Sforce virtual_force_goal, virtual_force_int_person, virtual_force_obstacle, virtual_force_robot;
	robot_->robot_propagation( dt_, parent_index , u.f );
	//check if the propagation is valid
	bool collision_check = check_collision( robot_->get_planning_trajectory()->back(), parent_index );

	// vertixes plotting
	//std::cout << robot_->get_robot_planning_trajectory()->back().x << " , " << robot_->get_robot_planning_trajectory()->back().y <<  std::endl;

	// Nearby people propagation and cost ---------------------------------------------------------------------
	SpointV_cov robot_point;
	for( Cperson_abstract* iit: nearby_person_list_ )
	{
		// 1 precalculation: if really close to the robot, then a complete propagation is done, otherwise
		// the correspondant Spose is sought.
		if ( !iit->is_needed_to_propagate_person_for_planning(  parent_index, robot_->get_robot_planning_trajectory()->back() , index_to_copy ) )
		{
			//update parent index correspondent vector TODO has to be an easiest way to not copy this
			iit->planning_propagation_copy( index_to_copy );
			robot_point =  robot_->get_planning_trajectory()->back();
			virtual_force_robot =  iit->force( robot_point , get_sfm_int_params(iit,robot_),
								&(iit->get_planning_trajectory()->at(parent_index) ) );

		}
		else
		{
			// 2 time step propagation
			virtual_force_goal = iit->force_goal( iit->get_best_dest() , get_sfm_params(robot_),
					&(iit->get_planning_trajectory()->at(parent_index) )  );
			virtual_force_int_person = force_persons_int_planning_virtual( iit, parent_index );
			if( read_force_map_success_ )
				virtual_force_obstacle = get_force_map( iit->get_planning_trajectory()->at(parent_index).x,
						iit->get_planning_trajectory()->at(parent_index).y ) * 0.5;

		    //obstacles due to laser scans. for real environments has priority over map forces
			if(read_laser_obstacle_success_)
				virtual_force_obstacle = force_objects_laser_int_planning_virtual( iit , parent_index,16.0);
			robot_point =  robot_->get_planning_trajectory()->back();
			virtual_force_robot =  iit->force( robot_point , get_sfm_int_params(iit,robot_),
					&(iit->get_planning_trajectory()->at(parent_index) ) );
			iit->set_forces_person( virtual_force_goal, virtual_force_int_person, virtual_force_robot, virtual_force_obstacle );
			iit->planning_propagation(dt_, iit->get_force_person() , parent_index);
		}
		//calculate the interaction forces cost TO BE DEPRECATED
		//dr = (Spoint)iit->get_planning_trajectory()->back() -
		//		(Spoint)iit->get_planning_trajectory()->at(parent_index);
		//work += fabs(virtual_force_robot * dr);

	}


	return collision_check;
}

void Cplan_local_nav::calculate_cost( unsigned int parent_index , Sedge_tree u )
{
	//for each propagation, a cost is calculated
	//robot cost
	Spoint dr;
	double cost(0.0);
	//dr = (Spoint) robot_->get_robot_planning_trajectory()->back() -
	//		(Spoint) robot_->get_robot_planning_trajectory()->at(parent_index);
	//work = fabs(u.f_goal * dr);
	cost = u.f_goal.module2(cost_angular_);
	cost_robot_.push_back( cost_robot_.at(parent_index) + cost);

	// Obstacles cost -----------------------------------------------------------------------------------------
	// If collision is detected, this function should return a high value,
	// but it does not distinguish if a collision took place, only an abnormally huge value
	//work = fabs(u.f_obs * dr );
	//cost_obstacles_.push_back( cost_obstacles_.at(parent_index) + work );

	//alternative way: max force module
	cost = u.f_obs.module2();
	//if ( cost_obstacles_.at(parent_index) > work )
		//work = cost_obstacles_.at(parent_index);
	cost_obstacles_.push_back( cost_obstacles_.at(parent_index) + cost );

	//people cost --------------------------------------------------------------------------------------------
	Sforce f,f_int;
	cost = 0;
	for( Cperson_abstract* iit: nearby_person_list_  )
	{
		switch( pr_force_mode_ )
		{
		case 0: //deterministic force, classical definition
			f = iit->force_sphe(robot_->get_planning_trajectory()->back() , get_sfm_int_params(iit,robot_) , &(iit->get_planning_trajectory()->at( parent_index )) );
			break;
		case 1: //probabilistic force, sampling around the elipsoid
			f = iit->force_sphe_prob( robot_->get_planning_trajectory()->back(), get_sfm_int_params(iit,robot_),&(iit->get_planning_trajectory()->at( parent_index )) );
			break;
		case 2: //probabilistic force, mahalanobis distance
			f = iit->force_sphe_mahalanobis( robot_->get_planning_trajectory()->back(), get_sfm_int_params(iit,robot_), &(iit->get_planning_trajectory()->at( parent_index )));
			break;
		case 3: //probabilistic force, worst case scenario: covaraince ellipsoid nearer to the center point
			f = iit->force_sphe_worst( robot_->get_planning_trajectory()->back(), get_sfm_int_params(iit,robot_), &(iit->get_planning_trajectory()->at( parent_index )));
			break;
		}
		f_int += f;
		cost += f.module2();
	}
	//if ( cost_int_forces_.at(parent_index) > work )
		//work = cost_int_forces_.at(parent_index);
	cost_int_forces_.push_back( cost_int_forces_.at(parent_index) + cost );


	//distance cost --------------------------------------------------------------------------------------------
	cost = robot_->get_planning_trajectory()->back().distance2(local_goal_);
	cost_distance_.push_back( cost_distance_.at(parent_index) + cost );
	double o = robot_->get_planning_trajectory()->back().angle_heading_point( local_goal_ );
	cost_orientation_.push_back( cost_orientation_.at(parent_index) + o*o );

    // Local minima cost  --------------------------------------------------------------------------------------------------------- TOBEDEPRECATED
    // identify possible local minima problems abs ( sum f ) != sum( abs f) Initial calculation
	//double sum_abs = u.f_goal.module2() + u.f_people.module2() + u.f_obs.module2();
	//double sum = (u.f_goal + u.f_people + u.f_obs).module2();
	//cost_local_minima_.push_back( cost_local_minima_.at(parent_index) - sum/sum_abs);//summation of all local minima indicators throughout the path


	nodes_in_branch_.push_back( nodes_in_branch_.at(parent_index) + 1.0 );

	// Simililarities wrt previous path --------------------------------------------------------------------------
	//this function is properly calculated using the COMPLETE path. see 8) in planning()
}

double Cplan_local_nav::cost_to_go( const Spoint& random_goal, unsigned int i  )
{
	double result;
	double d, t, o, dx, dy;
	//metrics to evaluate the nearest vertex
	switch( distance_mode_ )
	{
		//distance to random goal ----------------------- Euclidean distance: TO BE DEPRECATED   --------------------
		case Cplan_local_nav::Euclidean  :
			//1- calculate distance normalized to max distance
			d = robot_->get_robot_planning_trajectory()->at(i).distance( random_goal )
					/ workspace_radii_ * cost_parameters_[0];
			//2- orientation
			dx = random_goal.x - robot_->get_robot_planning_trajectory()->at(i).x;
			dy = random_goal.y - robot_->get_robot_planning_trajectory()->at(i).y;
			o = fabs(diffangle( robot_->get_robot_planning_trajectory()->at(i).theta ,
					atan2( dy, dx )  )) * cost_parameters_[1];

			//3- accumulated robot work cost_parameters_[2] DEPRECATED
			//wr = cost_robot_[i]*cost_parameters_[2];

			//4- accumulated persons work cost_parameters_[3] DEPRECATED
			//wp = cost_int_forces_[i]*cost_parameters_[3];

			//5- time not allowed being outside the time horizon
			t = (robot_->get_robot_planning_trajectory()->at(i).time_stamp -
					now_ ) / horizon_time_ ;
			//TODO other costs! bstacles + Local min
			if ( t > 1.0 ) t = 10000.0; //discard the vertex
			else t *= cost_parameters_[4];

			//6 - obstacles
			//wo = cost_obstacles_[i]*cost_parameters_[5];
			//- Local minima indicator
			//lm = cost_parameters_[7]*cost_local_minima_[i]/nodes_in_branch_[i];;
			//check for min
			result = d+o+t;
			break;

		//cost-to-go linear normalization to random goal  ------------------------------------------------------
		  case  Cplan_local_nav::Cost2go_norm :
		//cost-to-go erf normalization to random goal ----------------------------------------------------------
		  case  Cplan_local_nav::Cost2go_erf :
		  {
			SpointV robot = robot_->get_planning_trajectory()->at(i);
			Sforce f_goal, f_int, f_obs, f;
			double look_time(0.0),cost_robot(0.0),cost_distance(0.0), cost_int(0.0), cost_obs(0.0),cost_orientation(0.0),o;
			unsigned int prediction_index(2),max_index_prediction(0);
			//assert to avoid degenerated cases when the path does not look up to t_h due to being near the goal
			if ( robot_->get_planning_trajectory()->size() <  horizon_time_index_)
			{
				max_index_prediction = robot_->get_planning_trajectory()->size() -1;
			}
			else
			{
				max_index_prediction = horizon_time_index_;
			}
			//calculates current prediction time index [0,horizon] and distance to random goal of previous path, TODO not working well
			while(  look_time < robot.time_stamp  && prediction_index < max_index_prediction )
			{
				look_time = robot_->get_planning_trajectory()->at(prediction_index).time_stamp;
				cost_distance += robot_->get_planning_trajectory()->at(prediction_index).distance2( random_goal );//previous distance to get to the goal
				prediction_index += 2;
			}
			prediction_index -= 2;
			if ( look_time - now_ > horizon_time_ || prediction_index >=  horizon_time_index_ )
				return 1e10;
			//std::cout << "entering  " <<  i << "  time = " << prediction_index <<  "  size = " << robot_->get_planning_trajectory()->size() << std::endl;
			cost_robot = cost_robot_[i]/2.0;//as we are using double time step and half propagations, the accumulated robot cost
			cost_obs = cost_obstacles_[i]/2.0;
			cost_int = cost_int_forces_[i]/2.0;
			while ( prediction_index < horizon_time_index_  )
			{
				// cost due to interacting forces
				for( Cperson_abstract* iit: nearby_person_list_  )
				{
					if ( iit->get_prediction_trajectory()->at( prediction_index ).distance2(robot) < 16.0 )//<4m...to speed up, a more strict threshold is used
					{
						//TODO aqui algo no va bien, sale demasiado pequeña esta cantidad: error o propagacion fatal
						f = iit->force_sphe(robot , get_sfm_int_params(iit,robot_) , &iit->get_prediction_trajectory()->at( prediction_index ) );
						//std::cout <<  f.module2() << " , " <<  std::endl;
						f_int += f;
						cost_int += f.module2();
					}
				}
				// potential cost to go destination
				f_goal = robot_->force_goal( Sdestination(0,random_goal.x,random_goal.y), get_sfm_params(robot_), &robot );
				cost_robot += f_goal.module2(cost_angular_);

				//cost due to obstacles
				for( Spoint iit : laser_obstacle_list_)
				{
					if ( iit.distance2(robot) < 9.0 )
					{
						f = robot_->force_sphe( iit, get_sfm_int_params(robot_), &robot );
						f_obs += f;
						cost_obs += f.module2();
					}
				}

				// robot propagation
				robot = robot.propagate( dt_*2, f_goal+f_obs+f_int, robot_->get_desired_velocity() );//at double timestep to speed up calculations
				cost_distance += robot.distance2(random_goal);
				o = robot.angle_heading_point( random_goal );
				cost_orientation += o*o;
				prediction_index += 2;

			}
			if( distance_mode_ == Cplan_local_nav::Cost2go_erf )
			{
				//std::cout << "%cost_distance, cost_orientation, cost_robot, cost_int, cost_obs" << endl;
				//std::cout  << cost_distance << " , " << cost_orientation << " , " << cost_robot << " , " << cost_int << " , " <<  cost_obs << std::endl;
				result = cost_parameters_[0]*erf((cost_distance - mean_cost_distance_) / std_cost_distance_) +
						cost_parameters_[1]*erf((cost_orientation - mean_cost_orientation_) / std_cost_orientation_) +
						cost_parameters_[2]*erf((cost_robot - mean_cost_robot_) / std_cost_robot_ ) +
						cost_parameters_[3]*erf((cost_int - mean_cost_int_forces_) / std_cost_int_forces_ ) +
						cost_parameters_[5]*erf((cost_obs - mean_cost_obstacles_) / std_cost_obstacles_);
			}
			else
			{
			  //use the mean as the utopia point a std as the difference f_max - utopia, calculated previously
			result = cost_parameters_[0]*((cost_distance - mean_cost_distance_) / std_cost_distance_) +
						cost_parameters_[1]*((cost_orientation - mean_cost_orientation_) / std_cost_orientation_) +
						cost_parameters_[2]*((cost_robot - mean_cost_robot_) / std_cost_robot_ ) +
						cost_parameters_[3]*((cost_int - mean_cost_int_forces_) / std_cost_int_forces_ ) +
						cost_parameters_[5]*((cost_obs - mean_cost_obstacles_) / std_cost_obstacles_);
			}
			break;
		  }
		  //cost-to-go to a random goal only considering steering forces and collision ------------------------------------------------------
		  case  Cplan_local_nav::Cost2go_raw :
		  default:
		  {
			SpointV robot = robot_->get_planning_trajectory()->at(i);
			Sforce f_goal;
			double look_time(0.0), cost_robot(0.0);
			unsigned int prediction_index(2),max_index_prediction(0);
			//assert to avoid degenerated cases when the path does not look up to t_h due to being near the goal
			if ( robot_->get_planning_trajectory()->size() <  horizon_time_index_)
			{
				max_index_prediction = robot_->get_planning_trajectory()->size() -1;
			}
			else
			{
				max_index_prediction = horizon_time_index_;
			}
			//calculates current prediction time index [0,horizon] and distance to random goal of previous path, TODO not working well
			while(  look_time < robot.time_stamp  && prediction_index < max_index_prediction )
			{
				look_time = robot_->get_planning_trajectory()->at(prediction_index).time_stamp;
				prediction_index += 2;
			}
			prediction_index -= 2;
			if ( look_time - now_ > horizon_time_ || prediction_index >=  horizon_time_index_ )
				return 1e10;
			//std::cout << "entering  " <<  i << "  time = " << prediction_index <<  "  size = " << robot_->get_planning_trajectory()->size() << std::endl;
			cost_robot = cost_robot_[i]/2.0;//as we are using double time step and half propagations, the accumulated robot cost
			while ( prediction_index < horizon_time_index_  )
			{
				// potential cost to go destination
				f_goal = robot_->force_goal( Sdestination(0,random_goal.x,random_goal.y), get_sfm_params(robot_), &robot );
				cost_robot += f_goal.module2(cost_angular_);//fy cost 0.5

				// robot propagation
				robot = robot.propagate( dt_*2, f_goal, robot_->get_desired_velocity() );//at double timestep to speed up calculations
				prediction_index += 2;

				//check for collisions, only with obstacles
				if (  check_collision( robot ) )
					return 1e10;
			}
			result = cost_robot;

			break;
		  }
		}//end of switch


	return result;
}

unsigned int Cplan_local_nav::global_min_cost_index(   )
{
	double cost, min_cost(1e10);
	unsigned int min_cost_index(0);
	double d, o, dx, dy;
	preprocess_global_parameters();//depending on mode, needs a different preprocessing of data
	switch( global_mode_ )
	{

	  //distance to local goal, end of branch ----------------------- Euclidean distance: TO BE DEPRECATED
	  case Cplan_local_nav::Scalarization :
		for( unsigned int i : end_of_branches_index_ )
		{
			d = robot_->get_robot_planning_trajectory()->at(i).distance( local_goal_ );
			dx = goal_.x - robot_->get_robot_planning_trajectory()->at(i).x;
			dy = goal_.y - robot_->get_robot_planning_trajectory()->at(i).y;
			o = fabs(diffangle( robot_->get_robot_planning_trajectory()->at(i).theta ,
				atan2( dy, dx )  ));
			cost = cost_parameters_[0]*d +
				cost_parameters_[1]*o +
				cost_parameters_[2]*cost_robot_[i] +
				cost_parameters_[3]*cost_int_forces_[i]/nodes_in_branch_[i] +
				cost_parameters_[5]*cost_obstacles_[i]/nodes_in_branch_[i] +
				cost_parameters_[6]*cost_past_traj_[i];
			if ( cost < min_cost)
			{
				min_cost = cost;
				min_cost_index = i;
			}
			/*std::cout //<< "cost values (goal,ori,rob,int,obs,past,lm) = ("
				<< d <<
				" , " << o <<
				" , " << cost_int_forces_[i]/nodes_in_branch_[i] <<
				" , " << cost_obstacles_[i]/nodes_in_branch_[i] <<
				" , " << cost_past_traj_[i] <<
				" , " << cost_local_minima_[i]/nodes_in_branch_[i] << std::endl;*/
		}
		break;
		// global cost, estimation and normalization ---------------------------------------------------------------
	  case Cplan_local_nav::Weighted_sum_erf :
		for( unsigned int i : end_of_branches_index_ )
		{
			cost = cost_parameters_[0]*erf((cost_distance_[i] - mean_cost_distance_) / std_cost_distance_) +
				cost_parameters_[1]*erf((cost_orientation_[i] - mean_cost_orientation_) / std_cost_orientation_) +
				cost_parameters_[2]*erf((cost_robot_[i] - mean_cost_robot_) / std_cost_robot_ ) +
				cost_parameters_[3]*erf((cost_int_forces_[i] - mean_cost_int_forces_) / std_cost_int_forces_ ) +
				cost_parameters_[5]*erf((cost_obstacles_[i] - mean_cost_obstacles_) / std_cost_obstacles_) +
				cost_parameters_[6]*erf((cost_past_traj_[i] - mean_cost_past_traj_) / std_cost_past_traj_);
			if ( cost < min_cost)
			{
				min_cost = cost;
				min_cost_index = i;
			}
			/*std::cout << i << " ,"
			 	<< erf((cost_distance_[i] - mean_cost_distance_) / std_cost_distance_) << " , "
				<< erf((cost_orientation_[i] - mean_cost_orientation_) / std_cost_orientation_) << " , "
				<< erf((cost_robot_[i] - mean_cost_robot_) / std_cost_robot_ ) << " , "
				<< erf((cost_int_forces_[i] - mean_cost_int_forces_) / std_cost_int_forces_ ) << " , "
				<< erf((cost_obstacles_[i] - mean_cost_obstacles_) / std_cost_obstacles_) << std::endl;*/
		}
		break;
	  case  Cplan_local_nav::Weighted_sum_norm :
		  // mean_cost is the utopia cost corresponding to a free path and std_cost is the max_cost - utopia, according to MMO clasical approaches
	    for( unsigned int i : end_of_branches_index_ )
	    {
			cost = cost_parameters_[0]*((cost_distance_[i] - mean_cost_distance_) / std_cost_distance_) +
					cost_parameters_[1]*((cost_orientation_[i] - mean_cost_orientation_) / std_cost_orientation_) +
					cost_parameters_[2]*((cost_robot_[i] - mean_cost_robot_) / std_cost_robot_ ) +
					cost_parameters_[3]*((cost_int_forces_[i] - mean_cost_int_forces_) / std_cost_int_forces_ ) +
					cost_parameters_[5]*((cost_obstacles_[i] - mean_cost_obstacles_) / std_cost_obstacles_) +
					cost_parameters_[6]*((cost_past_traj_[i] - mean_cost_past_traj_) / std_cost_past_traj_);
			if ( cost < min_cost)
			{
				min_cost = cost;
				min_cost_index = i;
			}
			/*std::cout << i << " , "
				<<	((cost_distance_[i] - mean_cost_distance_) / std_cost_distance_) << " , "
				<< ((cost_orientation_[i] - mean_cost_orientation_) / std_cost_orientation_) << " , "
				<< ((cost_robot_[i] - mean_cost_robot_) / std_cost_robot_ ) << " , "
				<< ((cost_int_forces_[i] - mean_cost_int_forces_) / std_cost_int_forces_ ) << " , "
				<< ((cost_obstacles_[i] - mean_cost_obstacles_) / std_cost_obstacles_) << std::endl;*/
	    }
		break;
	  case  Cplan_local_nav::MO_erf :
	  case  Cplan_local_nav::MO_norm :
	  {
		  //first approach: push into a set of best trajectories
		  nondominated_plan_vertex_index_.clear();
		  nondominated_end_of_plan_vertex_index_.clear();
		  unsigned int nondominated_branch_index, cont;
		  for (  Smulticost m : nondominated_multicosts_)
		  {
		    nondominated_branch_index = m.id;
		    cont = (horizon_time_index_/2)*2;//to avoid plotting all solutions, we will only plot the nondominated set, even number

		    //print nondominated and normalized costs
		    //m.print_ml();
		    //print nondominated raw costs
			/*std::cout << nondominated_branch_index << " , "
				<< cost_distance_[nondominated_branch_index]  << " , "
				<< cost_orientation_[nondominated_branch_index]  << " , "
				<< cost_robot_[nondominated_branch_index]  << " , "
				<< cost_int_forces_[nondominated_branch_index]  << " , "
				<< cost_obstacles_[nondominated_branch_index]  << std::endl;*/
			nondominated_end_of_plan_vertex_index_.push_back( nondominated_branch_index );//only end of branches to plot the branch id
			do
			{
				nondominated_plan_vertex_index_.push_back( nondominated_branch_index);
				nondominated_branch_index = edge_[ nondominated_branch_index ].parent;
				--cont;
			}while( nondominated_branch_index > 0 && cont >= 1);
		  }

		  if( global_mode_ == Cplan_local_nav::MO_erf )
		  {
			  for( unsigned int i : nondominated_end_of_plan_vertex_index_)
			  {
				cost = cost_parameters_[0]*erf((cost_distance_[i] - mean_cost_distance_) / std_cost_distance_) +
						cost_parameters_[1]*erf((cost_orientation_[i] - mean_cost_orientation_) / std_cost_orientation_) +
						cost_parameters_[2]*erf((cost_robot_[i] - mean_cost_robot_) / std_cost_robot_ ) +
						cost_parameters_[3]*erf((cost_int_forces_[i] - mean_cost_int_forces_) / std_cost_int_forces_ ) +
						cost_parameters_[5]*erf((cost_obstacles_[i] - mean_cost_obstacles_) / std_cost_obstacles_) +
						cost_parameters_[6]*erf((cost_past_traj_[i] - mean_cost_past_traj_) / std_cost_past_traj_);
				if ( cost < min_cost)
				{
					min_cost = cost;
					min_cost_index = i;
				}
			  }
		  }
		  else
		  {

			for( unsigned int i : nondominated_end_of_plan_vertex_index_ )
			{
				cost = cost_parameters_[0]*((cost_distance_[i] - mean_cost_distance_) / std_cost_distance_) +
						cost_parameters_[1]*((cost_orientation_[i] - mean_cost_orientation_) / std_cost_orientation_) +
						cost_parameters_[2]*((cost_robot_[i] - mean_cost_robot_) / std_cost_robot_ ) +
						cost_parameters_[3]*((cost_int_forces_[i] - mean_cost_int_forces_) / std_cost_int_forces_ ) +
						cost_parameters_[5]*((cost_obstacles_[i] - mean_cost_obstacles_) / std_cost_obstacles_)+
						cost_parameters_[6]*((cost_past_traj_[i] - mean_cost_past_traj_) / std_cost_past_traj_);
				if ( cost < min_cost)
				{
					min_cost = cost;
					min_cost_index = i;
				}
		    }
		  }
		break;
	  }
	}
	//fill best cost
	best_costs_.resize( 5, 0.0 );
	best_costs_[0] = cost_distance_[min_cost_index];
	best_costs_[1] = cost_orientation_[min_cost_index];
	best_costs_[2] = cost_robot_[min_cost_index];
	best_costs_[3] = cost_int_forces_[min_cost_index];
	best_costs_[4] = cost_obstacles_[min_cost_index];
	//fill mean costs
	mean_costs_.resize( 5, 0.0 );
	mean_costs_[0] = mean_cost_distance_;
	mean_costs_[1] = mean_cost_orientation_;
	mean_costs_[2] = mean_cost_robot_;
	mean_costs_[3] = mean_cost_int_forces_;
	mean_costs_[4] = mean_cost_obstacles_;
	//fill std costs
	std_costs_.resize( 5, 0.0 );
	std_costs_[0] = std_cost_distance_;
	std_costs_[1] = std_cost_orientation_;
	std_costs_[2] = std_cost_robot_;
	std_costs_[3] = std_cost_int_forces_;
	std_costs_[4] = std_cost_obstacles_;
	return min_cost_index;
}

Sforce Cplan_local_nav::force_persons_int_planning_virtual(Cperson_abstract* center , unsigned int t, double min_dist2)
{
	Sforce force_res;
    for( auto iit : nearby_person_list_)
    {
        if( *center != *iit && center->get_planning_trajectory()->at(t)
                .distance2( iit->get_planning_trajectory( )->at(t) ) < min_dist2 )
        {
            force_res += center->force( iit->get_planning_trajectory( )->at(t) ,  get_sfm_int_params(center,iit),
            		&(center->get_planning_trajectory()->at(t) ) );
        }
    }
	return force_res;
}

Sforce Cplan_local_nav::force_persons_int_robot_prediction_virtual(const SpointV& center , unsigned int t, double min_dist2)
{
	//calculates the resultant forces to all nearby people due to the robot position
	Sforce force_res;
    for( auto iit : nearby_person_list_)
    {
        if( center.distance2( iit->get_planning_trajectory( )->at(t) ) < min_dist2 )
        {
            force_res += iit->force( iit->get_prediction_trajectory( )->at(t) ,  get_sfm_int_params(iit),
            		&center );
        }
    }
	return force_res;
}

Sforce Cplan_local_nav::force_objects_laser_int_planning_virtual( Cperson_abstract* person, unsigned int planning_index, double min_dist2, bool robot_collision_check_flag)
{
	Sforce force_res;
	const SpointV_cov* virtual_current_point;
	if ( planning_index == 0)
	{
		virtual_current_point = &(person->get_current_pointV());
	}
	else
	{
		virtual_current_point = &(person->get_planning_trajectory()->at(planning_index) );
	}
	nearby_obstacle_list_.clear();
	//for robot, checks
	for( Spoint iit : laser_obstacle_list_)
	{
		//there is a list for persons and another for robot(s). This function is for obstacles
		double d2 = person->get_current_pointV().distance2( iit );
		if( d2 < min_dist2)//square distance 5^2
		{
			force_res += person->force_sphe( iit , get_sfm_int_params(person), virtual_current_point );
			if( robot_collision_check_flag && d2 < 1.0 )//detect ultra nearby obstacles and check after propagation if collision took place
			{
				nearby_obstacle_list_.push_back( iit );
			}
		}
	}
	return force_res;

}

bool Cplan_local_nav::check_collision( const Spoint& p ,  int t)
{
	//TODO does not work properly on real scenarios! long term issue
	//check colision with map: TO BE DEPRECATED
	if ( read_force_map_success_ && !is_cell_clear_map( p.x, p.y ) )
//	if ( read_force_map_success_ && !is_cell_clear_map( robot_->get_robot_planning_trajectory()->back().x,	robot_->get_robot_planning_trajectory()->back().y ) )
	{
		return  true;
	}


	//check collision with obstacles (laser), prior is necessary to calculate force_objects_laser_int_planning()
	//for( Spoint iit : nearby_obstacle_list_ )//TODO not working properly since the c2g recalculates this vector
	for( Spoint iit : laser_obstacle_list_ )
	{
		if( iit.distance2( p )  < robot_->get_platform_radii_2() )
		{
			//std::cout << "Collision found!!" << robot_->get_platform_radii_2() << std::endl;
			return true;
		}
	}
	//if t = -1, then no collision is calculated wrt ppl
	if ( t > -1 )
	{
		double d,det;
		switch( ppl_collision_mode_ )
		{
		  case 0: //normal mode, quadratic distance to person, independent of distribution
			for( auto iit : nearby_person_list_)
			{
				d = iit->get_planning_trajectory( )->at(t).distance2( p );
				if ( d < robot_->get_platform_radii_2())
				{
					return true;
				}
			}
			break;
		  case 1://mahalanobis distance, considers collision if inside the std ellipsoid: TOO RESTRICTIVE...
			for( auto iit : nearby_person_list_)
			{
				d = iit->get_planning_trajectory( )->at(t).cov_dist(p,det);
				if ( d < 1.0 )
				{
					//std::cout << "Colision detected, pr distance = " << d << std::endl;
					return true;
				}
			}
			break;
		  case 2 ://mahalanobis distance, half the std, much less restrictive
			for( auto iit : nearby_person_list_)
			{
				d = iit->get_planning_trajectory( )->at(t).cov_dist(p,det);
				if ( det > 1.0)
				{
					if ( d < 0.5 )
					{
						return true;
					}
				}
				else
				{
					d = iit->get_planning_trajectory( )->at(t).distance2( p );
					if ( d < robot_->get_platform_radii_2() )
					{
						return true;
					}
				}
			}
			break;
		  case 3 :
			for( auto iit : nearby_person_list_)
			{
				d = iit->get_planning_trajectory( )->at(t).cov_dist(p,det);
				if ( det > 1.0)
				{
					if ( d < 0.3 )
					{
						return true;
					}
				}
				else
				{
					d = iit->get_planning_trajectory( )->at(t).distance2( p );
					if ( d < robot_->get_platform_radii_2() )
					{
						return true;
					}
				}
			}
			break;

		}

			//calculate probabilities. NOT implemented, insignificant collision values when cov is high
            //pr_no_col *= 1 - gaussian_constant_ / sqrt(det) * exp( -0.5*d );// * robot_->get_platform_radii_2();//area = pi * r^2 (pi included in gaussian_contant)
            //std::cout << "Pr of no colision joint = " << pr_no_col << " and distance = " << d << " det = " << det << std::endl;
	}

	return false;
}

void Cplan_local_nav::set_number_of_vertex( unsigned int n )
{
	max_iter_ = n;
    edge_.reserve((size_t)max_iter_);
    cost_robot_.reserve((size_t)max_iter_);
    cost_int_forces_.reserve((size_t)max_iter_);
    cost_obstacles_.reserve((size_t)max_iter_);
    //cost_local_minima_.reserve((size_t)max_iter_);
    cost_distance_.reserve((size_t)max_iter_);
    cost_orientation_.reserve((size_t)max_iter_);
    cost_past_traj_.reserve((size_t)max_iter_);
    nodes_in_branch_.reserve((size_t)max_iter_);
	random_goals_.reserve( (size_t)max_iter_ );


}

void Cplan_local_nav::preprocess_global_parameters()
{
	switch( global_mode_)
	{
	case Cplan_local_nav::Weighted_sum_erf :
		calculate_normalization_cost_functions_parameters_erf();
		break;
	case Cplan_local_nav::MO_erf :
		calculate_normalization_cost_functions_parameters_erf();
		calculate_non_dominated_solutions();
		break;
	case Cplan_local_nav::MO_norm :
		calculate_normalization_cost_functions_parameters_norm();
		calculate_non_dominated_solutions();
		break;
	case Cplan_local_nav::Scalarization :
	case Cplan_local_nav::Weighted_sum_norm :
		//calcualte normalization: we need utopia point and max value
		calculate_normalization_cost_functions_parameters_norm();
		break;
	}
}

void Cplan_local_nav::calculate_non_dominated_solutions()
{
	//fill the multiobjective cost structure
	multicosts_.clear();
	Smulticost m(0,5);
	bool is_candidate_dominated;
	for( unsigned int i : end_of_branches_index_ )
	{
		m.id = i;
		//raw costs
		/*m.cost[0] = cost_distance_[i];// [0] Goal cost
		m.cost[1] = cost_orientation_[i];// [1] orientation cost
		m.cost[2] = cost_robot_[i];// [2] Robot cost
		m.cost[3] = cost_int_forces_[i];// [3]Interacting people cost
		m.cost[4] = cost_obstacles_[i];// [5] obstacles cost*/
		//normalized costs
		m.cost[0] = (cost_distance_[i] - mean_cost_distance_) / std_cost_distance_;// [0] Goal cost
		m.cost[1] = (cost_orientation_[i] - mean_cost_orientation_) / std_cost_orientation_;// [1] orientation cost
		m.cost[2] = (cost_robot_[i] - mean_cost_robot_) / std_cost_robot_;// [2] Robot cost
		m.cost[3] = (cost_int_forces_[i] - mean_cost_int_forces_) / std_cost_int_forces_;// [3]Interacting people cost
		m.cost[4] = (cost_obstacles_[i] - mean_cost_obstacles_) / std_cost_obstacles_;// [5] obstacles cost
		multicosts_.push_back(m);
	}
	//calculate the non-dominated set
	nondominated_multicosts_.clear();
	for( Smulticost i : multicosts_  )
	{
		//i.print_ml();
		is_candidate_dominated = false;
		for( std::list<Smulticost>::iterator j = nondominated_multicosts_.begin() ; j != nondominated_multicosts_.end(); j++ )
		{
			//check it is not the same
			if ( i!=*j )
			{
				// check if i dominates j
				if ( i < *j )
				{
					//i.print();
					//j->print();
					j = nondominated_multicosts_.erase(j);
				}
				// check if j dominates i: break and look for a new candidate i
				else if ( *j < i )
				{
					is_candidate_dominated = true;
					break;
				}

				//else, nothing happens until all set is compared with i
			}
		}
		if( !is_candidate_dominated )
		{
			nondominated_multicosts_.push_back(i);
		}
	}
	//std::cout << "size of the non-dominated set = " << set.size() << " / " << end_of_branches_index_.size() << std::endl;
}

void Cplan_local_nav::calculate_normalization_cost_functions_parameters_erf()
{
	mean_cost_int_forces_=0.0;
	mean_cost_robot_=0.0;
	mean_cost_obstacles_=0.0;
	mean_cost_past_traj_=0.0;
	mean_cost_distance_=0.0;
	mean_cost_orientation_=0.0;
	double N = (double)end_of_branches_index_.size();
	for( unsigned int i : end_of_branches_index_ )
	{
		mean_cost_int_forces_ += cost_int_forces_[i];
		mean_cost_robot_ +=cost_robot_[i];
		mean_cost_obstacles_ +=cost_obstacles_[i];
		mean_cost_past_traj_ += cost_past_traj_[i];
		mean_cost_distance_ +=cost_distance_[i];
		mean_cost_orientation_ +=cost_orientation_[i];

		//plot of the entire raw costs
		/*std::cout << cost_int_forces_[i] << " , " <<
				cost_robot_[i] << " , " <<
				cost_obstacles_[i] << " , " <<
				cost_distance_[i] << " , " <<
				cost_orientation_[i] <<	std::endl;*/
	}
	mean_cost_int_forces_ /= N;
	mean_cost_robot_ /= N;
	mean_cost_obstacles_ /= N;
	mean_cost_past_traj_ /= N;
	mean_cost_distance_ /= N;
	mean_cost_orientation_ /= N;

	std_cost_int_forces_=0.0;
	std_cost_robot_=0.0;
	std_cost_obstacles_=0.0;
	std_cost_past_traj_=0.0;
	std_cost_distance_=0.0;
	std_cost_orientation_=0.0;
	double d;
	N -= 1.0;//unbiased std
	for( unsigned int i : end_of_branches_index_ )
	{
		d = cost_int_forces_[i] - mean_cost_int_forces_;
		std_cost_int_forces_ += d*d;
		d = cost_robot_[i] - mean_cost_robot_;
		std_cost_robot_ += d*d;
		d = cost_obstacles_[i] - mean_cost_obstacles_;
		std_cost_obstacles_ += d*d;
		d = cost_past_traj_[i] - mean_cost_past_traj_;
		std_cost_past_traj_ += d*d;
		d = cost_distance_[i] - mean_cost_distance_;
		std_cost_distance_ += d*d;
		d = cost_orientation_[i] - mean_cost_orientation_;
		std_cost_orientation_ += d*d;
	}
	if( std_cost_int_forces_ > 0.01 )
	{
		std_cost_int_forces_ /= N; std_cost_int_forces_ = sqrt(std_cost_int_forces_);
	}
	else
		std_cost_int_forces_ = 0.01;
	if( std_cost_robot_ > 0.01)
	{
		std_cost_robot_ /= N; std_cost_robot_ = sqrt(std_cost_robot_);
	}
	else
		std_cost_robot_ = 0.01;
	if( std_cost_obstacles_ > 0.01)
	{
		std_cost_obstacles_ /= N;std_cost_obstacles_ = sqrt(std_cost_obstacles_);
	}
	else
		std_cost_obstacles_ = 0.01;
	if( std_cost_past_traj_ > 0.01)
	{
		std_cost_past_traj_ /= N; std_cost_past_traj_= sqrt(std_cost_past_traj_);
	}
	else
		std_cost_past_traj_ = 0.01;
	if( std_cost_distance_ > 0.01)
	{
		std_cost_distance_ /= N; std_cost_distance_= sqrt(std_cost_distance_);
	}
	else
		std_cost_distance_ = 0.01;
	if( std_cost_orientation_ > 0.01)
	{
		std_cost_orientation_ /= N; std_cost_orientation_= sqrt(std_cost_orientation_);
	}
	else
		std_cost_orientation_ = 0.01;

	//printing results
	//std::cout << "cost_distance = (" << mean_cost_distance_ << " , " << std_cost_distance_ << std::endl;
	//std::cout << "cost_orientation = (" << mean_cost_orientation_ << " , " << std_cost_orientation_ << std::endl;
	//std::cout << "cost_int_forces = (" << mean_cost_int_forces_ << " , " << std_cost_int_forces_ << std::endl;
	//std::cout << "cost_robot = (" << mean_cost_robot_ << " , " << std_cost_robot_ << std::endl;
	//std::cout << "cost_obstacles = (" << mean_cost_obstacles_ << " , " << std_cost_obstacles_ << std::endl;
	//std::cout << "cost_past_traj = (" << mean_cost_past_traj_ << " , " << std_cost_past_traj_ << std::endl;
}

void Cplan_local_nav::calculate_normalization_cost_functions_parameters_norm()
{
	//calculates the Utopia point considering free space and the max value in order to linearly normalize ->[0,1]
	mean_cost_distance_=0.0;
	mean_cost_orientation_=0.0;
	mean_cost_robot_=0.0;
	mean_cost_int_forces_=0.0;
	mean_cost_obstacles_=0.0;
	mean_cost_past_traj_=0.0;
	//propagate the robot as if no obstacle is in the scene and calculate costs
	SpointV robot = robot_->get_planning_trajectory()->front();
	Sforce f_goal;
	while ( robot.time_stamp - now_ < horizon_time_ && robot.distance2( local_goal_ ) > 0.25 )
	{
		// potential cost to go destination
		f_goal = robot_->force_goal( Sdestination(0,local_goal_.x,local_goal_.y), get_sfm_params(robot_), &robot );

		// robot propagation
		robot = robot.propagate( dt_, f_goal, robot_->get_desired_velocity() );

		//calculate costs
		mean_cost_robot_ += f_goal.module2(cost_angular_);
		mean_cost_distance_ += robot.distance2(local_goal_);
		double o = robot.angle_heading_point( local_goal_ );
		mean_cost_orientation_ += o*o;
	}

	//calculate max costs and the scalarization term
	double max;
	max = -10000.0;
	for( unsigned int i : end_of_branches_index_ )
	{
		if (max < cost_distance_[i])
			max = cost_distance_[i];
	}
	std_cost_distance_ = max - mean_cost_distance_;
	if ( std_cost_distance_ < 0.01 ) std_cost_distance_ = 0.01;

	max = -10000.0;
	for( unsigned int i : end_of_branches_index_ )
	{
		if (max < cost_orientation_[i])
			max = cost_orientation_[i];
	}
	std_cost_orientation_ = max - mean_cost_orientation_;
	if ( std_cost_orientation_ < 0.01 ) std_cost_orientation_ = 0.01;

	max = -10000.0;
	for( unsigned int i : end_of_branches_index_ )
	{
		if (max < cost_robot_[i])
			max = cost_robot_[i];
	}
	std_cost_robot_ = max - mean_cost_robot_;
	if ( std_cost_robot_ < 0.01 ) std_cost_robot_= 0.01;

	max = -10000.0;
	for( unsigned int i : end_of_branches_index_ )
	{
		if (max < cost_int_forces_[i])
			max = cost_int_forces_[i];
	}
	std_cost_int_forces_ = max - mean_cost_int_forces_;
	if ( std_cost_int_forces_ < 0.01 ) std_cost_int_forces_ = 0.01;

	max = -10000.0;
	for( unsigned int i : end_of_branches_index_ )
	{
		if (max < cost_obstacles_[i])
			max = cost_obstacles_[i];
	}
	std_cost_obstacles_ = max - mean_cost_obstacles_;
	if ( std_cost_obstacles_ < 0.01 ) std_cost_obstacles_ = 0.01;

	max = -10000.0;
	for( unsigned int i : end_of_branches_index_ )
	{
		if (max < cost_past_traj_[i])
			max = cost_past_traj_[i];
	}
	std_cost_past_traj_= max - mean_cost_past_traj_;
	if ( std_cost_past_traj_< 0.01 ) std_cost_past_traj_= 0.01;

	//printing results
	//std::cout << "cost_distance = (" << mean_cost_distance_ << " , " << std_cost_distance_ << std::endl;
	//std::cout << "cost_orientation = (" << mean_cost_orientation_ << " , " << std_cost_orientation_ << std::endl;
	//std::cout << "cost_int_forces = (" << mean_cost_int_forces_ << " , " << std_cost_int_forces_ << std::endl;
	//std::cout << "cost_robot = (" << mean_cost_robot_ << " , " << std_cost_robot_ << std::endl;
	//std::cout << "cost_obstacles = (" << mean_cost_obstacles_ << " , " << std_cost_obstacles_ << std::endl;
	//std::cout << "cost_past_traj = (" << mean_cost_past_traj_ << " , " << std_cost_past_traj_ << std::endl;
}

Spose Cplan_local_nav::get_best_planned_pose(double dt)
{
	//returns the best planned pose at time dt
	int index(0);
	if ( dt < dt_)
	    index = 1;
	else
	    index = (int) (dt / dt_);
	if( !best_plan_vertex_index_.empty() && (unsigned)index < best_plan_vertex_index_.size() )
	{
		unsigned int next_plan_index = best_plan_vertex_index_.at( best_plan_vertex_index_.size() - index );
		return robot_->get_robot_planning_trajectory()->at(next_plan_index);
	}
	else
		return Spose();
}

void Cplan_local_nav::
get_navigation_instant_work( double& work_robot, double& work_persons )
{
	work_robot = work_robot_;
	work_persons = work_persons_;
}


void Cplan_local_nav::
calculate_navigation_instant_work( )
{
	work_robot_ = 0.0;
	if( best_plan_vertex_index_.size() > 2 )
	{
		work_robot_ =  fabs( edge_.at(best_plan_vertex_index_.at( best_plan_vertex_index_.size()-2 )).f  *
			robot_->get_diff_position() );
	}
	else
	{
		work_robot_ = 0.0;
	}

	work_persons_ = 0.0;
	for( Cperson_abstract* iit : nearby_person_list_)
	{
		if ( iit->get_current_pointV().distance( robot_->get_current_pointV() ) < workspace_radii_ )
			work_persons_ += fabs( iit->force( robot_->get_current_pointV() ,
					get_sfm_int_params(iit,robot_) ) * iit->get_diff_position() );
	}
}


void Cplan_local_nav::set_robot_params( double v, double w, double av, double av_break, double aw, double platform_radii)
{
	robot_->set_v_max( v );
	max_v_by_system_ = v;//this velocity is modified depending on the density of nearby obstacles
	robot_->set_w_max( w );
	robot_->set_a_v_max( av );
	robot_->set_a_v_break( av_break );
	robot_->set_a_w_max( aw );
	robot_->set_platform_radii( platform_radii);
}

//TODO update to new cost-to-go function and elimiate DEPRECATED parameters
void Cplan_local_nav::set_plan_cost_parameters( double c_dist, double c_orientation, double c_w_robot,
		double c_w_people, double c_time, double c_w_obstacles, double c_old_path, double c_l_minima)
{
    cost_parameters_[0] = c_dist;// [0] Goal cost
    cost_parameters_[1] = c_orientation;// [1] orientation cost
    cost_parameters_[2] = c_w_robot;// [2] Robot cost
    cost_parameters_[3] = c_w_people;// [3]Interacting people cost
    cost_parameters_[4] = c_time;// [4] potential time
    cost_parameters_[5] = c_w_obstacles;// [5] obstacles cost
    cost_parameters_[6] = c_old_path;// [6] past function cost
    cost_parameters_[7] = c_l_minima;// [7] local minima scape cost
}

//only use when no plan is calculated, as an evaluation of the observed performance
// of this method or any navigation method
void Cplan_local_nav::calculate_navigation_cost_values( std::vector<double>& costs )
{
	costs.resize(4,0.0);
	// calculate robot cost --------------------------------------------------------------------------------
	Spose dr = robot_->get_diff_pose();
	costs[0] = dr.v* dr.v + dr.w*dr.w;
	costs[0] /= dt_*dt_;


	// calculate ppl cost --------------------------------------------------------------------------------
	Sforce f,f_int;
	double cost = 0.0;
	for( Cperson_abstract* iit: person_list_  )
	{
		f = iit->force_sphe(robot_->get_current_pointV() , get_sfm_int_params(iit,robot_)  );
		f_int += f;
		cost += f.module2();
	}
	costs[1] = cost;

	// calculate obstacles cost  --------------------------------------------------------------------------------
	Sforce f_obs = force_objects_laser_int_planning_virtual( robot_, 0, 25.0, false );//false set for no new nearby_obstacles list to be refilled
	costs[2] = f_obs.module2();


	// calculate average velocity --------------------------------------------------------------------------------
	costs[3] = robot_->get_current_pose().v;

}

Sedge_tree::Sedge_tree(	unsigned int parent_, Sforce f_, Sforce f_goal_,
		Sforce f_people_,Sforce f_obs_):
		parent(parent_), f(f_), f_goal(f_goal_), f_people(f_people_), f_obs(f_obs_)
{

}
double Sedge_tree::cost( Sedge_tree parent) const
{
	//TODO
	return 0.0;
}
void Sedge_tree::print()
{
	std::cout << "parent vertex = " << parent << std::endl;
	f.print();
}

Smulticost::Smulticost( unsigned int id_, unsigned int n ) :
	id(id_)
{
	cost.resize( n, 0.0 );
}

bool Smulticost::operator< ( const Smulticost& m2) const
{
	//dominance operator. Returns true if m dominates m2
	double thr(1e-5);
	bool one_better(false);
	for( unsigned int i = 0; i<cost.size(); ++i )
	{
		//for numerical stability, equality is cheked:  x-x2 < thr
		if ( cost[i] - thr >  m2.cost[i] )
			return false;
		// check for at least one better cost i
		else if ( cost[i] + thr <  m2.cost[i] )
			one_better = true;
		//else: equal both costs, needs more comparisons

	}
	return one_better;//requires that at least one was better
}

bool Smulticost::operator== ( const Smulticost& m2) const
{
	if( id == m2.id )
		return true;
	else
		return false;
}


bool Smulticost::operator!= ( const Smulticost& m2) const
{
	return !(*this==m2);
}

void Smulticost::print ( ) const
{
	std::cout << "Multicost " << id << ", costs = {";
	for( double i: cost )
		std::cout << i << " , ";
	std::cout << " } " << std::endl;
}

void Smulticost::print_ml ( ) const
{
	std::cout << id ;
	for( double i: cost )
		std::cout << ", "<< i;
	std::cout << std::endl;
}
