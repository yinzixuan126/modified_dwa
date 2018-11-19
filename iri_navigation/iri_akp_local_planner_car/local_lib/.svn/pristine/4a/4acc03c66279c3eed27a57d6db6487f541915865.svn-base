/*
 * force_reactive_robot_companion_learning.h
 *
 *  Created on: Nov 12, 2013
 *      Author: gferrer
 */

#ifndef FORCE_REACTIVE_ROBOT_COMPANION_LEARNING_H_
#define FORCE_REACTIVE_ROBOT_COMPANION_LEARNING_H_


#include "nav/force_reactive_robot_companion.h"

class Cforce_reactive_robot_companion_learning : public Cforce_reactive_robot_companion
{
  public:
	Cforce_reactive_robot_companion_learning();
	~Cforce_reactive_robot_companion_learning();
	std::vector<double> learning_force_companion_params_MC( );
	std::vector<double> learning_force_companion_params_alphabeta( );


  protected:
	double alpha_std_, beta_std_,gamma_std_, delta_std_;
	std::vector<double> previous_params_;
	double alpha_,beta_;
	long int cont_;
};


#endif /* FORCE_REACTIVE_ROBOT_COMPANION_LEARNING_H_ */
