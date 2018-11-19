/*
 * personReal.h
 *
 *  Created on: Jul 9, 2013
 *      Author: gferrer
 */

#ifndef PERSON_BHMIP_H_
#define PERSON_BHMIP_H_

#include "scene_elements/person_abstract.h"

class Cperson_bhmip : public Cperson_abstract
{
  public:
	Cperson_bhmip(unsigned int id=0,
			Cperson_abstract::target_type person_target_type=Cperson_abstract::Person,
			Cperson_abstract::force_type person_force_type=Cperson_abstract::Spherical,
			double _time_window = 1.0);
	virtual ~Cperson_bhmip();
	//const std::deque<SpointV_cov>* get_trajectory() {return &trajectory_;}
	//std::deque<std::vector<double> >& get_phi_prob() { return phi_prob_; }
	//std::deque<std::vector<double> >& get_phi_pose2dest() { return phi_pose2dest_; }
	virtual void reset();
	virtual void add_pointV( SpointV_cov point,
			Cperson_abstract::filtering_method filter=Cperson_abstract::Linear_regression_filtering);
	virtual void refresh_person( double now );
	virtual void prediction(  double min_v_to_predict );

	virtual void rotate_and_translate_trajectory(double R, double thetaZ,
			double linear_vx, double linear_vy, double v_rot_x, double v_rot_y, std::vector<double> vect_odom_eigen_tf, bool debug_odometry = false); // For local tracker.

  private:
	void trajectory_windowing( );
	std::deque<SpointV_cov> trajectory_;
	double time_window_; //0.5 segs
	SpointV_cov filter_current_state_linear_regression( );
	SpointV_cov filter_current_state_linear_regression_bayes( );
	SpointV_cov low_pass_filter_current_state_linear_regression( );
	void  intention_precalculation();
	std::vector<double> posterior_destinations_prob_;
	double phi_var_;
	std::deque<std::vector<double> > phi_pose2dest_ , phi_prob_;
	double time_from_last_update_;


	// variables for local tracker
	std::deque<std::vector<double>> hom2_tf;
	std::vector<double> hom2_ant;
	std::deque<double> trajectory_local_velocity_x_;
	std::deque<double> trajectory_local_velocity_y_;
};


#endif /* PERSON_BHMIP_H_ */
