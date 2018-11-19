/*
 * personReal.h
 *
 *  Created on: Dec 17, 2013
 *      Author: gferrer
 */

#ifndef PERSON_BEHAVIOR_H_
#define PERSON_BEHAVIOR_H_

#include "scene_elements/person_abstract.h"
#include "scene_elements/person_bhmip.h"
#include <list>



class Cperson_behavior : public Cperson_bhmip
{
  public:
	Cperson_behavior(unsigned int id=0,
			Cperson_abstract::target_type person_target_type=Cperson_abstract::Person,
			Cperson_abstract::force_type person_force_type=Cperson_abstract::Spherical,
			double _time_window = 1.0);
	virtual ~Cperson_behavior();
	virtual void reset();
	virtual void add_pointV( SpointV_cov point,
			Cperson_abstract::filtering_method filter=Cperson_abstract::Linear_regression_filtering);
	virtual void prediction( double min_v_to_predict );
	/**
	 * this virtual method calculates interaction forces depending on the
	 * expected behavior, that is, changing the force parameters for each
	 * different person
	 */
	virtual Sbehavior* find_behavior_estimation( unsigned int id );
	virtual Cperson_abstract::behavior_type	get_best_behavior_to_person( unsigned int interacting_person ) const;

	//methods for trajectory prediction
	virtual void prediction_propagation( double dt , Sforce force = Sforce(), unsigned int index = 0 );
	virtual void planning_propagation( double dt , Sforce force = Sforce(), unsigned int index = 0 );
	virtual void planning_propagation_copy( unsigned int prediction_index );
	virtual bool is_needed_to_propagate_person_for_planning( unsigned int parent_index, Spoint robot, unsigned int& new_index_to_be_copied );
	virtual void reset_propagation_flag() { has_copied_propagation_ = false; }
	virtual const std::vector<SpointV_cov>* get_planning_trajectory() const { return &planning_trajectory_;}//raw planning trajectory
	virtual const std::vector<SpointV_cov>* get_prediction_trajectory() const {  return &prediction_trajectory_;}//predicted trajectory, according to best plan
	virtual void clear_prediction_trajectory();
	virtual void clear_planning_trajectory();
	virtual void reserve_prediction_trajectory( unsigned int n );



  private:
	std::vector<SpointV_cov> planning_trajectory_, prediction_trajectory_;//container for predictions and planning
	bool has_copied_propagation_;
	std::list<Sbehavior> expected_behavior_list_;


};


#endif /* PERSON_BEHAVIOR_H_ */
