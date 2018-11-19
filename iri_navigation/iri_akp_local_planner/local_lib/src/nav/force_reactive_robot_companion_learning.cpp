/*
 * force_reactive_robot_companion_learning.cpp
 *
 *  Created on: Nov 12, 2013
 *      Author: gferrer
 */



#include "nav/force_reactive_robot_companion_learning.h"
#include "random/rand_gmm.h"
#include "math.h"


Cforce_reactive_robot_companion_learning::Cforce_reactive_robot_companion_learning():
alpha_std_(0.1), beta_std_(0.1), gamma_std_(3.0), delta_std_(0.5), alpha_(0.0),beta_(1.0),cont_(0)
{
	previous_params_.push_back( 0.7 );//alpha
	previous_params_.push_back( 0.3 );//beta
	previous_params_.push_back( 5.0 );//interaction persons
	previous_params_.push_back( 0.5 );//obstacles laser
	previous_params_.push_back( 0.5 );//obstacles maps

	//set random seed for the rng
	rand_seed();

}

Cforce_reactive_robot_companion_learning::~Cforce_reactive_robot_companion_learning()
{

}

std::vector<double> Cforce_reactive_robot_companion_learning::learning_force_companion_params_MC( )
{
	//propose new robot companion parameters using MC
	//settings are ready for new experiments, but has to be activated somewhere

	//5- set next param_previous
	std::vector<double> params = param_force_approach_;//parameters returning the below work


	//4- sample params depending on previous params, only gamma and delta are set
	double gamma, delta;
	//while ( alpha <= 0.0 || alpha > 2.0 )
	//	alpha = rand_normal( previous_params_[0] ,alpha_std_);
	while ( gamma <= 1.0 || gamma > 15.0 )
		gamma = rand_normal( previous_params_[2] ,4*gamma_std_);
	while ( delta <= 0.1 || delta > 1.0 )
		delta = rand_normal( previous_params_[4] ,4*delta_std_);
	//param_force_approach_[0] = alpha;
	//param_force_approach_[1] = beta;
	param_force_approach_[2] = gamma;
	param_force_approach_[4] = delta;

	return params;
}

std::vector<double> Cforce_reactive_robot_companion_learning::learning_force_companion_params_alphabeta( )
{
	std::vector<double> params = param_force_approach_;

	cont_++;
	if ( cont_ > 10 )//5 times per parameters value
	{
		cont_ = 0;
		alpha_ += 0.02;
		if ( alpha_ > 1.0 ) alpha_ = 1.0;
		beta_ = 1 - alpha_;
	}
	params[0] = alpha_; //alpha
	params[1] = beta_; //beta
	param_force_approach_ = params;
	return params;
}
