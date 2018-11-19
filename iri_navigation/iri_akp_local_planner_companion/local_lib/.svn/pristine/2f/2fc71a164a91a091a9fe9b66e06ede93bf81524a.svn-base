/*
 * robot.h
 *
 *  Created on: Jul 31, 2013
 *      Author: gferrer
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include "scene_elements/person_abstract.h"


/**
 *
 * \brief Robot class
 *
 * The robot class inherints from Cperson_prediction and thus it can use of the
 * calculation for the interaction with other scene elements.
 *
 * Addtionally, it is specifically designed to describe different robot propagations
 * such as a differential platform, a balancing differential (segway RMP200) or others.
 * It is intended to serve as a container for the planning algorithms described in this library.
 *
*/

class Crobot : public Cperson_abstract
{
  public:
	enum robot_type{Differential, Diff_balancing, Car};
	Crobot(  unsigned int id=0, robot_type robot_type=Differential,
			Cperson_abstract::force_type person_force_type=Cperson_abstract::Spherical,
			double _time_window = 1.0);
	virtual ~Crobot();
	// Cperson methods for updating robot position
	virtual void add_pointV( SpointV_cov point,
			Cperson_abstract::filtering_method filter=Cperson_abstract::Linear_regression_filtering) {}//not used in this class
	virtual void refresh_person( double now ) {} //this method is not used in Robot
	virtual void add_pose( Spose observation );
	Spose get_current_pose() { return current_pose_; }
	Spose get_diff_pose() { return diff_pose_;}
	// prediction
	virtual void prediction( double min_v_to_predict );
	virtual void reset();

	//methods for trajectory prediction, inherited form Cperson_abstract
	virtual const std::vector<SpointV_cov>* get_prediction_trajectory() const { return &planning_SpointV_trajectory_;}
	virtual const std::vector<SpointV_cov>* get_planning_trajectory() const { return &planning_SpointV_trajectory_;}
	virtual void clear_planning_trajectory();
	virtual void reserve_planning_trajectory( unsigned int n) { planning_trajectory_.reserve(n);}

	//specific robot planning methods and data structures functions
	const std::vector<Spose>* get_robot_planning_trajectory() const { return &planning_trajectory_;}
	void erase_last_planning_propagation();
	//cinematic propagation
	void robot_propagation(double dt, unsigned int index = 0, double v=1.0, double w=1.0 );
	//dynamic propagation
	void robot_propagation(double dt, unsigned int index = 0 , const Sforce& f = Sforce() );
	void set_rnd_local_goal( Spoint goal ) { rnd_goal_ = goal; }

	void correct_state_to_delay( Spose last_control_cmd, double now, double delay );

	//paramters
	void set_v_max( double v) { v_max_ = v; this->set_desired_velocty(v);}
	double get_v_max () { return v_max_; }
	void set_w_max( double w) { w_max_ = w;}
	double get_w_max () { return w_max_; }
	void set_a_v_max( double av) { a_v_max_ = av;}
	double get_a_v_max () { return a_v_max_; }
	void set_a_v_break( double av) { a_v_break_ = av;}
	double get_a_v_break () { return a_v_break_; }
	void set_a_w_max( double aw) { a_w_max_ = aw;}
	double get_a_w_max () { return a_w_max_; }
	double get_platform_radii() { return platform_radii_; }
	double get_platform_radii_2() { return platform_radii_2_;}
	void set_platform_radii( double r ) { platform_radii_ = r; platform_radii_2_ = r*r;}


	//behavior estimation methods
	virtual Sbehavior * find_behavior_estimation( unsigned int id );
	virtual Cperson_abstract::behavior_type get_best_behavior_to_person( unsigned int interacting_person ) const;


  private:
	//robot parameters
	Spose current_pose_, diff_pose_;
	double v_max_, w_max_, a_v_max_, a_v_break_, a_w_max_;
	double platform_radii_, platform_radii_2_;
	std::vector<Spose> planning_trajectory_;
	std::vector<SpointV_cov> planning_SpointV_trajectory_;
	Spoint rnd_goal_;

	//behavior containers
	std::list<Sbehavior> expected_behavior_list_;



};

#endif /* PERSON_ROBOT_H_ */
