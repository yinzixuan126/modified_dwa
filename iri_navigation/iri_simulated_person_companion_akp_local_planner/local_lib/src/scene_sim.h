/*
 * scene_sim.h
 *
 *  Created on: Mar 27, 2013
 *      Author: gferrer
 */

#ifndef SCENE_SIM_H_
#define SCENE_SIM_H_

#include "scene_abstract.h"
#include "scene_elements/person_abstract.h"
#include "iri_geometry.h"


/**
 *
 * \brief Cscene_simulated Class
 *
 * The scene class provides a set of methods to store, manage,
 * get and set information of a simulated scene.
 *
 */
class Cscene_sim : public Cscene_abstract
{
	public:
		Cscene_sim();
		virtual ~Cscene_sim();
		//needs a void observation for timestamp
		virtual void update_scene(const std::vector<SdetectionObservation>& observation );
		void set_number_virtual_people( int number_people ) { number_virtual_people_ = number_people; }
		void set_remove_targets( bool remove_targets){remove_targets_=remove_targets;}
		virtual void clear_scene();
		virtual Cperson_abstract* add_person_container( unsigned int id,
				std::list<Cperson_abstract*>::iterator iit);

	protected:
		unsigned int number_virtual_people_, virtual_person_id_counter_;
		bool remove_targets_;
};



#endif /* SCENE_SIM_H_ */
