#ifndef prediction_behavior_H
#define prediction_behavior_H

#include "prediction_bhmip.h"
#include "scene_elements/person_abstract.h"
#include "iri_geometry.h"
#include <vector>
#include <list>


/**
 *
 * \brief Prediction_behavior Class
 *
 * The prediction class provides a set of algorithms
 * estimating the behavior of persons interacting in
 * the scene as well as, and since it inherits from prediction_bhmip,
 * solving intentionality prediction and long term prediction.
 * A set of destinations is assumed to be provided
 *
 *
*/
class Cprediction_behavior: public Cprediction_bhmip
{
	public:
		Cprediction_behavior ( double horizon_time = 0.5, bool behavior_flag = false,
				Cperson_abstract::filtering_method filter= Cperson_abstract::No_filtering,
				Cperson_abstract::update_person_method update_method = Cperson_abstract::Autoremove);
		virtual ~Cprediction_behavior();
		virtual Cperson_abstract* add_person_container( unsigned int id,
				std::list<Cperson_abstract*>::iterator iit );
		/**
		 * \brief this function updates the behavior estimation and calculates a predicted trajectory
		 * function
		 */
		void scene_prediction();
		virtual void clear_scene();
	    void set_horizon_time( double t );
	    double get_horizon_time(  ) { return horizon_time_;}
	    void set_behavior_flag( bool flag ) { behavior_flag_ = flag;}
	    bool get_behavior_flag( ) {return behavior_flag_;}
	    const std::vector<double>* get_behavior_sfm_params( Cperson_abstract::behavior_type best_behavior_type );
		virtual const std::vector<double>* get_sfm_int_params( const Cperson_abstract * center_person,
				const Cperson_abstract * interacting_person = NULL);

		// calculate current forces, for visualization purposes
		void calculate_current_forces( int pr_force_mode = 0 );



	protected:
		void scene_behavior_estimation();
		void scene_trajectory_prediction();
		//calculation of forces for persons that do not exists, just propagated points
		Sforce force_persons_int_virtual( Cperson_abstract* center, unsigned long n=0);
		Sforce force_objects_laser_int_prediction( Cperson_abstract* person, unsigned int planning_index=0);
		void behavior_update( Cperson_abstract* observed_person, Cperson_abstract* center_person, const Sforce& observed_int);
		double horizon_time_;//max time of prediction
		unsigned int horizon_time_index_;
		bool behavior_flag_;
		std::vector<double> force_params_balanced_, force_params_aware_ , force_params_unaware_;



};
#endif
