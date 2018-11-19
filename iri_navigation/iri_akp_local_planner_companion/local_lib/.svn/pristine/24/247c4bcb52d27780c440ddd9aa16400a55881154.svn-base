#ifndef person_abstract_H
#define person_abstract_H


#include <deque>
#include <vector>
#include <list>
#include "iri_geometry.h"
#include <stddef.h>
#include <assert.h>



/**
 *
 * \brief Behavior Class
 *
 * The behavior class is a container class  for the different behavior paramters
 * used for a specific human motion prediction (Cperson_behavior)
 *
*/
class Sbehavior
{
  public:
	Sbehavior( unsigned int id );
	~Sbehavior();
	unsigned int related_person_id;
	std::vector<double> expectation;
	void print() const;
};

/**
 *
 * \brief Person Class 
 *
 * The person class is an abstract container class defining the interfaces
 * of the person prediction and trajectory storing (if needed)
 * 
 * different types of persons are defined: Person , Robot , Obstacle
 * depending on the prediction method or scene type
 *
*/
class Cperson_abstract
{
	public:
		enum target_type { Person=0 , Robot , Obstacle};
		enum force_type { Spherical=0, Elliptical, Collision_Prediction};
		enum behavior_type { Balanced=0, Aware, Unaware, Count_behaviors};
	    enum filtering_method{ No_filtering=0, Linear_regression_filtering, Bayes_filtering, Low_pass_linear_regression_filtering};
	    //Manual remove requires an external call to eliminate a person by using the scene method ()
	    enum update_person_method { Autoremove=0, Manual_remove };
		Cperson_abstract (  unsigned int id=0 ,
				Cperson_abstract::target_type target_type=Cperson_abstract::Person,
				Cperson_abstract::force_type person_force_type=Cperson_abstract::Spherical,
				double _time_window = 1.0);
		virtual ~Cperson_abstract()=0;
		unsigned int get_id() const { return person_id_; }
		void set_id( unsigned int id ) { person_id_ = id; }
		bool operator== (Cperson_abstract& p2) const;
		bool operator!= (Cperson_abstract& p2) const { return !(*this==p2); }

		virtual void add_pointV( SpointV_cov point ,
				Cperson_abstract::filtering_method filter=Cperson_abstract::Linear_regression_filtering ) = 0;
		virtual void refresh_person( double now ) = 0;
		virtual void prediction( double min_v_to_predict) = 0;
		void set_best_dest(const Sdestination& dest ){ best_destination_=dest; }
		Sdestination get_best_dest(){ return best_destination_; }
		void set_destinations( const std::vector<Sdestination>& dest ) { 	destinations_ = dest; }
		const std::vector<Sdestination>* get_destinations()   { return &destinations_; }
		void set_desired_velocty(double v){ desired_velocity_ = v;}
		double get_desired_velocity() { return desired_velocity_; }
		/**
			 * \brief point propagation
			 *
			 * this method propagates person position dt time in the future.
			 * if no destinations are set (or the best destination estimated is null)
			 * then propagates linearly with velocity as a short term predictor.
			*/
		SpointV_cov pointV_propagation( double dt , Sforce force = Sforce() );
		SpointV_cov pointV_propagation( double dt , Sforce force, SpointV_cov point );

		const SpointV_cov& get_current_pointV() const { return current_pointV_; }
		double get_time() { return current_pointV_.time_stamp; }

		//misc methods
		virtual void reset()=0;
		void print();
		void print_dest();
		void set_person_type( target_type type ) { type_ = type; }
		target_type get_person_type () const { return type_; }

		//forces calculation
		/**
			 * \brief force_cp
			 *
			 *  Calculates the interacting forces using the collision prediction model
			 *  described by Zanlungo et al in "" , 2011.
			 *
			*/
		Sforce force( const SpointV& interacting_person , const std::vector<double>* social_forces_param,
				const SpointV* virtual_current_point = NULL );
		Sforce force_sphe( const Spoint& interacting_person , const std::vector<double>* social_forces_param,
				const SpointV* virtual_current_point = NULL );
		Sforce force_ellip( const SpointV& interacting_person ,  const std::vector<double>* social_forces_param,
				const SpointV* virtual_current_point = NULL  );
		Sforce force_cp( const SpointV& interacting_person , const std::vector<double>* social_forces_param,
				const SpointV* virtual_current_point = NULL );
		Sforce force_goal( const Sdestination& dest , const std::vector<double>* social_forces_param, const SpointV* virtual_current_point = NULL );
	    Sforce force_sphe_prob( const SpointV_cov& interacting_person , const std::vector<double>* social_forces_param,
	            const SpointV* virtual_current_point = NULL );
	    Sforce force_sphe_mahalanobis( const SpointV_cov& interacting_person , const std::vector<double>* social_forces_param,
	            const SpointV* virtual_current_point = NULL );
	    Sforce force_sphe_worst( const SpointV_cov& interacting_person , const std::vector<double>* social_forces_param,
	            const SpointV* virtual_current_point = NULL );
		Sforce get_force_person() { return force_to_goal_ + force_int_person_ + force_int_robot_ + force_obstacle_; }
		Sforce get_forces_person( Sforce&  force_to_goal, Sforce& force_int_person , Sforce& force_int_robot, Sforce& force_obstacle ) const;
		Sforce get_force_int_robot_person() { return force_int_robot_; }
		void set_forces_person( Sforce  force_to_goal, Sforce force_int_person , Sforce force_int_robot, Sforce force_obstacle  );
		void set_int_force( Sforce force_int ) { force_int_person_ = force_int;}
		void set_int_robot_force( Sforce force_int ) { force_int_robot_ = force_int;}
		void set_obstacle_force( Sforce force_int ) { force_obstacle_ = force_int;}
		SpointV_cov get_diff_position(){ return diff_pointV_;}
		bool is_observation_updated(){ return observation_update_;}
		void set_observation_update(bool in){ observation_update_ = in; }

		//behavior estimation methods
		virtual Sbehavior * find_behavior_estimation( unsigned int id ) { return NULL;}
		virtual Cperson_abstract::behavior_type get_best_behavior_to_person( unsigned int interacting_person ) const
			{ return Cperson_abstract::Balanced;}

		//methods for trajectory prediction
		virtual void prediction_propagation( double dt , Sforce force = Sforce(), unsigned int index = 0 ) {}
		virtual void planning_propagation( double dt , Sforce force = Sforce(), unsigned int index = 0 ) {}
		virtual void planning_propagation_copy( unsigned int prediction_index ) {}
		virtual bool is_needed_to_propagate_person_for_planning( unsigned int parent_index, Spoint robot, unsigned int& new_index_to_be_copied) {return true;}
		virtual void reset_propagation_flag() {}
		virtual const std::vector<SpointV_cov>* get_prediction_trajectory() const { return NULL;}//carefull not to use this method if not prepared for predictions
		virtual const std::vector<SpointV_cov>* get_planning_trajectory() const { return NULL;}
		virtual void clear_prediction_trajectory() { }
		virtual void clear_planning_trajectory() { }
		virtual void reserve_planning_trajectory( unsigned int n) { }


		virtual void rotate_and_translate_trajectory(double R, double thetaZ, double linear_vx,
				double linear_vy, double v_rot_x, double v_rot_y, std::vector<double> vect_odom_eigen_tf, bool debug_odometry = false){}; // For local tracker


	protected:
		unsigned int person_id_;
		SpointV_cov current_pointV_;
		SpointV diff_pointV_;
		Sdestination best_destination_;
		std::vector<Sdestination> destinations_; //local person destinations. prior probabilities
		double desired_velocity_;
		target_type type_;//enum target_type { Person=0 , Robot , Obstacle, Virtual_Person};
		force_type person_force_type_;
		bool observation_update_;
		Sforce force_to_goal_, force_int_person_ , force_obstacle_, force_int_robot_;
		double now_;



};

//global functions to calculate forces
Sforce force_sphe( const SpointV& center_person, const SpointV& interacting_person , const std::vector<double>* social_forces_param );
Sforce force_goal( const Sdestination dest, const SpointV& center_person , const std::vector<double>* social_forces_param );


#endif
