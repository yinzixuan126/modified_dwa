/*
 * force_reactive_robot_companion.h
 *
 *  Created on: Nov 9, 2013
 *      Author: gonzalo
 */

#ifndef FORCE_REACTIVE_ROBOT_COMPANION_H_
#define FORCE_REACTIVE_ROBOT_COMPANION_H_

#include "prediction_bhmip.h"
/**
 *
 * \brief Force reactive Robot companion Class
 *
 * The force_reactive_robot_companion class provides an algorithm, making use of
 * intentionality and motion human prediction in order to control
 * the most human-like approach of the robot towards a moving person.
 * The approach algorithm is based on forces calculation and
 * an input feedback is expected in order to weight the importance
 * of the interacting forces.
 *
 * This work was published in "xxx IROS'2013"
 *
 *
*/


class Cforce_reactive_robot_companion : public Cprediction_bhmip
{
	public:
		Cforce_reactive_robot_companion ();
		~Cforce_reactive_robot_companion();
		enum approach_state { UNKNOWN_ZONE=0, INNER_ZONE , MID_ZONE, OUT_ZONE };
		SpointV_cov robot_approach_pose(bool stop_if_smn_in_inner=false);
		SpointV_cov robot_approach_pose( Sforce& f_goal, Sforce& f_persongoal,
				Sforce& f_int_pers, Sforce& f_int_laser, Sforce& f_int_map, Sforce& f, bool stop_if_smn_in_inner=false);
		//temporal function while no global localization is set (yet...)
		void set_target_person(int id) { target_person_id_ = id;}
		bool set_nearest_target_person();
		int get_target_person() { return target_person_id_; }
		Cperson_abstract* nearest_person( Cperson_abstract* person);
		double distance_to_nearest_person(Cperson_abstract* person);
		approach_state get_robot_state() { return robot_state_; }
		void set_v_max(double v_max){v_max_ = v_max;}
		double get_v_max(){return v_max_;}
		void set_v_cruise(double v){v_cruise_ = v;}
		void set_time_horizon(double t){time_horizon_ = t;}
		std::vector<double> get_force_params() const { return param_force_approach_; }
		void set_force_params( double force_goal, double force_toperson);
		void set_force_params( double force_goal, double force_toperson, double force_interaction);
		void set_force_params( double force_goal, double force_toperson,
					double force_interaction, double force_laser, double force_map);
		void robot_parameters_feedback_adjustment( double feedback_sign );
		void set_companion_position(double r, double ro){r_=r; ro_=ro;}
		bool get_is_target_person_visible () { return is_target_person_visible_;}

	protected:
		Sforce robot_force_persongoal(SpointV_cov robot, SpointV_cov person, std::vector<double> social_forces_param);
		int target_person_id_;
		std::vector<double> param_force_approach_;
		approach_state robot_state_;//state with respect to its approaching target
		void set_robot_state(SpointV_cov person, SpointV_cov robot, double lambda);
		SpointV_cov robot_tangential_propagation( Sforce f );

		double v_max_;//[m/s]
		double v_cruise_;//[m/s]
		double time_horizon_;//[s]
		double v_robot_desired_;//[m/s]
		double r_,ro_;//relative robot companion position in polar coordinates
		Sdestination person_destination_;
		bool is_target_person_visible_;

};


#endif /* FORCE_REACTIVE_ROBOT_COMPANION_H_ */
