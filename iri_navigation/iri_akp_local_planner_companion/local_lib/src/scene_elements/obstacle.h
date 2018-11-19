/*
 * obstacle.h
 *
 *  Created on: Jun 19, 2013
 *      Author: gferrer
 */

#ifndef OBSTACLE_H_
#define OBSTACLE_H_


#include "iri_geometry.h"
#include "scene_elements/person_abstract.h"

class Cobstacle : public Cperson_abstract {
public:
	Cobstacle( Spose position );
	~Cobstacle();
	virtual void reset();
	virtual void add_pose(){ }
	virtual void prediction(double min_v_to_predict){}

private:

};


#endif /* OBSTACLE_H_ */
