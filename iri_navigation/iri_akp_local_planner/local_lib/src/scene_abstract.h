#ifndef scene_abstract_H
#define scene_abstract_H

#include "scene_elements/person_abstract.h"
#include "iri_geometry.h"
#include "scene_elements/robot.h"
#include <list>
#include <vector>


/**
 *
 * \brief Scene_abstract Class
 *
 * The scene class provides an abstract interface and a set
 * of methods to store, manage,
 * get and set information regarding the moving targets present
 * in a given scene. It also manages the social-forces relative
 * to each of the members present in Cscene
 *
 */

class Cscene_abstract
{
	public:
	    enum person_container_type { Person_Virtual=0, Robot, Obstacle, Person_BHMIP};
		Cscene_abstract( Cperson_abstract::filtering_method filter= Cperson_abstract::No_filtering,
				Cperson_abstract::update_person_method update_method = Cperson_abstract::Autoremove,
				Cperson_abstract::force_type type=Cperson_abstract::Spherical );
		virtual ~Cscene_abstract()=0;
		void set_destinations ( std::vector<Sdestination>& dest );
		const std::vector<Sdestination>* get_destinations() { return &destinations_; }
		void set_dt( double dt ) { dt_ = dt; }
		double get_dt() { return dt_; }
		double get_time(){ return now_;}
		Cperson_abstract* add_person( unsigned int id );//private
		//, Cperson_abstract::target_type type=Cperson_abstract::Person,
		//		Cperson_abstract::force_type f_type=Cperson_abstract::Spherical );
		virtual Cperson_abstract* add_person_container( unsigned int id,
				std::list<Cperson_abstract*>::iterator iit )=0;
		void remove_person( unsigned int id );
		virtual void update_robot( Spose  observation );
		const Crobot* get_robot() const { return robot_;}
		void set_robot_destinations( std::vector<Sdestination>& dest ){ robot_->set_destinations( dest );}
		/**
		 * \brief get_scene
		 *
		 * This abstract function is the main method to update a scene  and
		 * its implementation depends on the class of scene.
		 */
		virtual void update_scene(const std::vector<SdetectionObservation>& observation ) = 0;
		const std::list<Cperson_abstract*>* get_scene() { return &person_list_; }
		bool find_person(unsigned int id);
		bool find_person(unsigned int id , Cperson_abstract** person);
		bool find_person(unsigned int id , Cperson_abstract** person, std::list<Cperson_abstract*>::iterator& it);
		void print();
		void set_scene_force_type( Cperson_abstract::force_type type ){ scene_force_type_ = type;}
		Cperson_abstract::force_type get_scene_force_type( ) { return scene_force_type_;}

		// Sforce methods
		Sforce force_persons_int( Cperson_abstract* person );
		Sforce force_objects_laser_int( Cperson_abstract* person );//this function calculates the repulsion forces due to static obstacles
		//void calculate_social_forces_all_persons();//to be deprecated
		virtual const std::vector<double>* get_sfm_int_params( const Cperson_abstract * center_person,
				const Cperson_abstract * interacting_person = NULL);
		const std::vector<double>* get_sfm_params( const Cperson_abstract * center_person );
		virtual void clear_scene()=0;

		//read laser
		void read_laser_scan( const std::vector<Spoint>&  laser_scan );//this is the only public function for processing laser scans
		const std::vector<Spoint>* get_laser_obstacles() { return &laser_obstacle_list_; }

		//read map
		bool read_force_map( const char * path );
		Sforce get_force_map( double x, double y );
		Sforce get_force_map_robot_position( ){
			return get_force_map( robot_->get_current_pointV().x, robot_->get_current_pointV().y );}
		bool is_cell_clear_map( double x, double y );
		void get_map_params( float &min_x, float &max_x, float &min_y, float &max_y, float &resolution ,
				unsigned int &map_number_x, unsigned int &map_number_y);
		bool get_map_obstacle(unsigned int i) { return obstacle_map_.at(i); }
		//temporal
		unsigned int xy_to_m(double x, double y);
		Spoint m_to_xy( unsigned int m);
		bool read_destination_map( const char * path );

		//force parameters methods
		void set_sfm_to_person( const std::vector<double>& params ){ social_forces_param_to_person_ = params;}
		void set_sfm_to_robot( const std::vector<double>& params ){ social_forces_param_to_robot_ = params;}
		void set_sfm_to_obstacle( const std::vector<double>& params ){ social_forces_param_to_obs_ = params;}


		// local tracker methods
		virtual void rotate_and_traslate_scene(unsigned int id, double R, double thetaZ, double linear_vx, double linear_vy, double v_rot_x, double v_rot_y, bool debug  = false){}


	protected:
		std::vector<Sdestination> destinations_;//prior scene destinations (all of them)
		Crobot* robot_;//robot is in a different list than normal persons, and additionally we have its pointer for specific robot methods
		std::list<Cperson_abstract *> person_list_;//list of persons in the scene, abstract containers
		double dt_;
		double now_;
		std::vector<Spoint> laser_obstacle_list_;//list of obstacles seen by laser
		bool read_laser_obstacle_success_;
		//std::vector<Spoint> map_obstacle_list_;//TODO: static list of obstacles according to map
		std::vector<Sforce> force_map_;
		std::vector<bool> obstacle_map_;
		//temporal occupancy matrix
		std::vector<Spoint> xy_map_;
		bool read_force_map_success_;
		unsigned int map_number_x_,map_number_y_;
		float min_x_,max_x_,min_y_,max_y_,map_resolution_;
		bool read_destination_map_success_;
		Cperson_abstract::force_type scene_force_type_;
		std::vector<double> social_forces_param_to_person_, social_forces_param_to_robot_, social_forces_param_to_obs_;
		Cperson_abstract::filtering_method filtering_method_;
		Cperson_abstract::update_person_method update_method_;
		bool robot_in_the_scene_;


		// read laser private methods
		bool point_corresponds_person( Spoint laser_point);
		bool point_belongs_to_laser_obstacle( Spoint p);

};
#endif

