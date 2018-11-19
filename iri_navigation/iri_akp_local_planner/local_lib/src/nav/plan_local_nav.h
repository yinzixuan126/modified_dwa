/*
 * plan_local_nav.h
 *
 *  Created on: Dec 22, 2013
 *      Author: gferrer
 */

#ifndef PLAN_LOCAL_NAV_H_
#define PLAN_LOCAL_NAV_H_

#include "prediction_behavior.h"
#include "scene_elements/person_abstract.h"
#include <random>

/**
 *	\brief Sedge_tree Struct
 *
 *	data struct describing a tree edge containing the index to the parent vertex
 *	and the command inputs (v.w) or forces required to get from that vertex state
 */
class Sedge_tree
{
  public:
	Sedge_tree(	unsigned int parent, Sforce f_=Sforce(), Sforce f_goal_=Sforce(),
			Sforce f_people_=Sforce(),Sforce f_obs_=Sforce());
	unsigned int parent;
	Sforce f;
	Sforce f_goal;
	Sforce f_people;
	Sforce f_obs;
	double cost(Sedge_tree parent)const;
	void print();
};

/**
 *	\brief Smulticost Struct
 *
 *	data struct for containing multiciple costs and compare them in a fast and clean way
 *	specially to determine dominance relations between solutions.
 */
class Smulticost
{
  public:
	Smulticost( unsigned int id_ = 0, unsigned int n = 1 );
	std::vector<double> cost;
	unsigned int id;
	bool operator< ( const Smulticost& m2) const;
	bool operator==( const Smulticost& m2) const;
	bool operator!=( const Smulticost& m2) const;
	void print() const;
	void print_ml() const;
};

/**
 *	\brief Cplan_local_nal
 *
 * Basic local planner using social forces
 *
 */
class Cplan_local_nav : public Cprediction_behavior
{
  public:
    enum plan_mode{  F_RRT_Uniform=0,  F_RRT_Gauss_Circle, F_RRT_GC_alpha};
    enum distance_mode{  Euclidean=0,  Cost2go_erf, Cost2go_norm , Cost2go_raw};
    enum global_mode{ Scalarization=0, Weighted_sum_erf, Weighted_sum_norm, MO_erf, MO_norm};
    /**
     * \brief Initial algorithm parameters in the constructor of the
     * algorithm
     */
    Cplan_local_nav(double horizon_time = 3.0, unsigned int max_iter = 1000,
    		plan_mode mode= Cplan_local_nav::F_RRT_Uniform);
    virtual ~Cplan_local_nav();

    /**
     * \brief this is the public interface to calculate the robot plan
     * according to the observed scene and the algorithm parameters.
     */
    bool robot_plan(Spose& pose_command, double dt = 0.0 );
    double get_distance_to_goal() { return robot_->get_current_pointV().distance( goal_ );}
    /**
     * Configuration methods
     */
    void set_planning_mode( plan_mode mode ) {plan_mode_ = mode;}
    plan_mode get_planning_mode(){ return plan_mode_; }
    void set_distance_mode( distance_mode mode )
    	{distance_mode_ = mode; nondominated_plan_vertex_index_.clear(); nondominated_end_of_plan_vertex_index_.clear();}
    distance_mode get_distance_mode(){ return distance_mode_;}
    void set_global_mode( global_mode mode )
    	{global_mode_ = mode; nondominated_plan_vertex_index_.clear();nondominated_end_of_plan_vertex_index_.clear();}
    global_mode get_global_mode(){ return global_mode_;}
    void set_ppl_collision_mode( int ppl_col ) {ppl_collision_mode_ =  ppl_col; }
    void set_pr_force_mode( int pr_force_mode ) { pr_force_mode_ = pr_force_mode;}
    /**
     * \brief set_robot_goal() function sets the value of the goal position to
     * which the planning aims to. It is assumed that a global planner, above this
     * implementation, provides these goals
     */
    void set_robot_goal(const  Spoint& goal );
    Spoint get_robot_goal() { return goal_;}
    Spoint get_robot_local_goal() { return local_goal_;}
    void set_plan_cost_parameters( double c_dist, double c_orientation, double c_w_robot,
    		double c_w_people, double c_time, double c_w_obstacles, double c_old_path, double c_l_minima);
    const std::vector<double>* get_plan_cost_parameters() const { return &cost_parameters_;}
    void set_number_of_vertex( unsigned int n );
    unsigned int get_number_of_vertex(  ) { return max_iter_;}
    void set_robot_params( double v, double w, double av, double av_break,double aw, double platform_radii);
    void set_xy_2_goal_tolerance( double tol) { xy_2_goal_tolerance_ = tol; }
    void set_v_goal_tolerance( double tol) { v_goal_tolerance_ = tol; }
    void set_distance_to_stop( double d ){ distance_to_stop_ = d; }
    double get_distance_to_stop( ) { return distance_to_stop_;}

    /**
     * Return info methods
     */
    const std::vector<unsigned int>* get_robot_plan_index() const { return &best_plan_vertex_index_;}
    const std::vector<unsigned int>* get_robot_nondominated_plan_index() const { return &nondominated_plan_vertex_index_;}
    const std::vector<unsigned int>* get_robot_nondominated_end_of_plan_index() const { return &nondominated_end_of_plan_vertex_index_;}
    const std::vector<Sedge_tree>* get_plan_edges() const { return &edge_;}
    const std::vector<Spoint>* get_random_goals() const { return &random_goals_;}
    double get_workspace_radii() { return workspace_radii_;}
    Spose get_best_planned_pose(double dt=0.0);
    void get_navigation_instant_work( double& work_robot, double& work_persons );
    void get_navigation_cost_values(  std::vector<double>& costs )//returns the vector of bests costs, calculated in the global_cost routine
    	{ costs = best_costs_;}
    void get_navigation_mean_cost_values(  std::vector<double>& costs ) { costs = mean_costs_;}
    void get_navigation_std_cost_values(  std::vector<double>& costs ) { costs = std_costs_;}
	/**
	 * \brief calculate scene cost
	 *
	 * This function calculates the costs due to robot navigation and
	 * nearby moving people. It is used only as an observation of the scene,
	 * no calculations are done, is just for evaluating purposes.
	 */
    void calculate_navigation_cost_values( std::vector<double>& costs );

  protected:
    /**
	 * \brief this function calculates the planner path, if no solution is
	 * found, then a false result is returned
     */
    bool robot_plan_anticipative_krrt( );
    /**
	 * \brief this function initializes the planner, selecting the persons to be considered
	 * and other things. If setting is not possible, i.e. goal is within current goal, etc..
	 * a false result is returned
     */
    bool init_robot_plan();
    /**
     * \brief this function samples the workspace
     */
    Spoint sample_workspace();
    void reset_scene_persons_propagation_flag();
    unsigned int find_nearest_vertex( const Spoint& random_goal );
    /**
     * \brief this function calculates the input required
     * for the propagation to get the sampled position
     */
    Sedge_tree calculate_edge(unsigned int parent_vertex_index, const Sdestination& random_goal);
    /**
     * \brief this function propagates the selected node,
     * if a collision is detected returns TRUE and if propagation
     * was not possible, false
     */
    bool propagate_vertex( unsigned int parent_vertex_index , const  Sedge_tree& u);
    /**
     * \brief this function checks for collision for the propagated robot
     * state either with static obstacles as well as persons (their prediction)
     */
    bool check_collision( const Spoint& p,  int t = -1 );
    double cost_to_go( const Spoint& random_goal, unsigned int i );
    Sforce force_persons_int_planning_virtual(Cperson_abstract* center , unsigned int t = 0, double min_dist2= 64.0);
    Sforce force_persons_int_robot_prediction_virtual(const SpointV& center , unsigned int t = 0, double min_dist2 = 16.0);
    Sforce force_objects_laser_int_planning_virtual( Cperson_abstract* person, unsigned int planning_index = 0, double min_dist2=25.0, bool robot_collision_check_flag=false);
    void calculate_navigation_instant_work(  );

    void calculate_cost( unsigned int parent_index , Sedge_tree u );
    //preprocess different solutions to calculate the best or bests solutions
    unsigned int global_min_cost_index(  );
    void preprocess_global_parameters();
    void calculate_normalization_cost_functions_parameters_erf();
    void calculate_normalization_cost_functions_parameters_norm();
    void calculate_non_dominated_solutions();

    //planning paramters
    plan_mode plan_mode_;
    distance_mode distance_mode_;
    global_mode global_mode_;
    unsigned int max_iter_;
    std::vector<double> cost_parameters_;
    Spoint goal_,local_goal_;
    bool reaching_goal_;
	//list of person to be considered in by the planner
	std::list<Cperson_abstract*> nearby_person_list_;
	std::vector<Spoint> nearby_obstacle_list_;
	double workspace_radii_, std_goal_workspace_;
	double max_v_by_system_, cost_angular_;
	double xy_2_goal_tolerance_, v_goal_tolerance_, local_v_goal_tolerance_, distance_to_stop_;

    // alpha and sum_ratio paramter to scape local minima
    double alpha_,gamma_,delta_;


    //random generator
	std::default_random_engine generator_;


    //tree structure data structures
    std::vector<Sedge_tree> edge_;
    std::vector<double> cost_int_forces_;
    std::vector<double> cost_robot_;
    std::vector<double> cost_obstacles_;
    std::vector<double> cost_past_traj_;
    //std::vector<double> cost_local_minima_;
    std::vector<double> nodes_in_branch_;
    std::vector<double> cost_distance_;
    std::vector<double> cost_orientation_;
    std::vector<Spoint> random_goals_;
    std::vector<bool> collision_detected_;
    double work_persons_, work_robot_;

    //results data structures
    std::vector<unsigned int> best_plan_vertex_index_, end_of_branches_index_, nondominated_plan_vertex_index_, nondominated_end_of_plan_vertex_index_;
    std::vector<Spose> best_planning_trajectory_;
    std::vector<double> best_costs_, mean_costs_, std_costs_;

    //cost-to-go function values to measure real performance for learning and testing
    std::vector<double> cost_values_;

    Spose last_pose_command_;//comand of the last velocity command send to the robot platform


    //Multi-Objective optimization: normalization terms
    double mean_cost_int_forces_,
		mean_cost_robot_,
		mean_cost_obstacles_,
		mean_cost_past_traj_,
		mean_cost_distance_,
		mean_cost_orientation_;

    double std_cost_int_forces_,
		std_cost_robot_,
		std_cost_obstacles_,
		std_cost_past_traj_,
		std_cost_distance_,
		std_cost_orientation_;

    //Multi-objective vector structure to find non-dominated sets
    std::vector<Smulticost> multicosts_;
    std::list<Smulticost> nondominated_multicosts_;

    //to calculate probability of collision
    double gaussian_constant_;
    int ppl_collision_mode_,pr_force_mode_;


};


#endif /* PLANN_LOCAL_NAV_H_ */



