#ifndef prediction_bhmip_H
#define prediction_bhmip_H

#include "scene_abstract.h"
#include "scene_elements/person_abstract.h"
#include "iri_geometry.h"
#include <vector>
#include <list>


/**
 *
 * \brief Prediction_bhmip Class
 *
 * The prediction class provides a set of algorithms
 * solving intentionality prediction and long term prediction.
 * A set of destinations is assumed to be provided
 *
 *
*/
class Cprediction_bhmip : public Cscene_abstract
{
	public:
		Cprediction_bhmip ( Cperson_abstract::filtering_method filter= Cperson_abstract::No_filtering,
				Cperson_abstract::update_person_method update_method = Cperson_abstract::Autoremove );
		virtual ~Cprediction_bhmip();
		virtual void update_scene(const std::vector<SdetectionObservation>& observation );
		virtual void rotate_and_traslate_scene(unsigned int id, double R, double thetaZ, double linear_vx, double linear_vy, double v_rot_x, double v_rot_y, std::vector<double> vect_odom_eigen_tf, bool debug  = false);
		virtual Cperson_abstract* add_person_container( unsigned int id,
				std::list<Cperson_abstract*>::iterator iit );
		/**
		 * scene_intentionality_prediction_bhmip( ) calculates the intentionality prediction
		 * for observed trajectories
		 */
		void scene_intentionality_prediction_bhmip( );
		virtual void clear_scene();

		void set_min_v_to_predict( double min_v_to_predict ) { min_v_to_predict_ =  min_v_to_predict;}
		double get_min_v_to_predict() { return min_v_to_predict_;}

		void set_filtering_time_window( double time_window) {filtering_time_window_ = time_window;}
		double get_filtering_time_window( void ) {return filtering_time_window_;}




	protected:
		double min_v_to_predict_;
		double filtering_time_window_;



};
#endif
