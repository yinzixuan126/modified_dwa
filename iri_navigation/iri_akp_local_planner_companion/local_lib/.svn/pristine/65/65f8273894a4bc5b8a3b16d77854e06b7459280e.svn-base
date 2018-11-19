/*
 * person_virtual.h
 *
 *  Created on: Jul 9, 2013
 *      Author: gferrer
 */

#ifndef PERSON_VIRTUAL_H_
#define PERSON_VIRTUAL_H_

#include "scene_elements/person_abstract.h"

/**
 *
 * \brief Person_virtual Class
 *
 * The person virtual class is designed to contain the minimum amount of information
 * required to observe a person. No intentionality prediction is done (it is assumed known).
 *
 * It is mainly designed for simulations and for propagating an observed scene, thereby
 * requiring the minimum amount of calculations to propagate.
 *
 * different types of persons are also available: Person , Robot , Obstacle
 *
*/
class Cperson_virtual : public Cperson_abstract
{
  public:
	Cperson_virtual( unsigned int id=0 ,
			Cperson_abstract::target_type person_target_type=Cperson_abstract::Person,
			Cperson_abstract::force_type person_force_type=Cperson_abstract::Spherical,
			double _time_window = 1.0);
	virtual ~Cperson_virtual();
	virtual void add_pointV( SpointV_cov point,
			Cperson_abstract::filtering_method filter=Cperson_abstract::Linear_regression_filtering );
	virtual void refresh_person( double now ) {} //this method is not used when Virtual people
	virtual void prediction( double min_v_to_predict);
	virtual void reset();

  private:


};

#endif /* PERSON_VIRTUAL_H_ */
