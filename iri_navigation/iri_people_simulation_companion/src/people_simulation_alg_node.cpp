#include "people_simulation_alg_node.h"
#include <pluginlib/class_list_macros.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

PeopleSimulationAlgNode::PeopleSimulationAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<PeopleSimulationAlgorithm>(),
  n_persons_(0), simulation_mode_( PeopleSimulationAlgNode::Normal ),
  tf_listener_(ros::Duration(10.f)),ini_companion_(true),
  fscan_received(false), 
  rscan_received(false),//,
  debug_antes_subgoals_entre_AKP_goals_(false),
  debug_real_test_companion_(false),
  initialized_(true),
  person_companion_slicing_path_diff_orientation_(20.0),
  text_markers_old_size_(0),
  debug_person_companion_(true),
  vis_mode2_(2)
  //tf_listener2_(NULL)
{
  //init class attributes if necessary
  this->scene_.set_bool_sim(true);
  this->loop_rate_ = 10;//in [Hz]

  // [init publishers]
  this->tracksMarkers_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("tracksMarkers", 100);
  this->tracks_publisher_ = this->public_node_handle_.advertise<iri_perception_msgs::detectionArray>("tracks", 100);
  pthread_mutex_init(&this->reset_mutex_,NULL);
  this->personCompanionAkpMarkers_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("person_companion_akp_planner_markers", 100);

// [init subscribers]
  this->fscan_subscriber_   = this->public_node_handle_.subscribe<sensor_msgs::LaserScan>("front_scan" , 1, boost::bind(&PeopleSimulationAlgNode::fscan_callback, this, _1));
  this->rscan_subscriber_   = this->public_node_handle_.subscribe<sensor_msgs::LaserScan>("rear_scan" , 1,  boost::bind(&PeopleSimulationAlgNode::rscan_callback, this, _1));
  pthread_mutex_init(&this->fscan_mutex2_,NULL);
  pthread_mutex_init(&this->rscan_mutex2_,NULL);

  // [init services]
  this->reset_server_ = this->public_node_handle_.advertiseService("reset", &PeopleSimulationAlgNode::resetCallback, this);
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
 
  init_sim();
}

void PeopleSimulationAlgNode::fscan_mutex_enter2(void){
  pthread_mutex_lock(&this->fscan_mutex2_);
}
void PeopleSimulationAlgNode::fscan_mutex_exit2(void){
  pthread_mutex_unlock(&this->fscan_mutex2_);
}
void PeopleSimulationAlgNode::rscan_mutex_enter2(void){
  pthread_mutex_lock(&this->rscan_mutex2_);
}
void PeopleSimulationAlgNode::rscan_mutex_exit2(void){
  pthread_mutex_unlock(&this->rscan_mutex2_);
}

PeopleSimulationAlgNode::~PeopleSimulationAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->reset_mutex_);
  pthread_mutex_destroy(&this->fscan_mutex2_);
  pthread_mutex_destroy(&this->rscan_mutex2_);
}

void PeopleSimulationAlgNode::init_sim()
{
 ROS_INFO("IN PeopleSimulationAlgNode::init_sim()");
  this->fixed_frame = "/map"; // (ely) variable to include fake laser (fake obstacles) in people simulation

  scene_.set_dt( 0.2 );
	//this->public_node_handle_.getParam("simulation_mode", simulation_mode_);
  this->public_node_handle_.getParam("number_persons", n_persons_);
  this->public_node_handle_.getParam("force_map_path", force_map_path_);
  this->public_node_handle_.getParam("destination_map_path", destination_map_path_);
  this->public_node_handle_.getParam("remove_targets", remove_targets_);
  this->public_node_handle_.getParam("freeze", freeze_);
  this->public_node_handle_.getParam("robot", robot_);

	scene_.set_number_virtual_people( n_persons_ );
  scene_.set_remove_targets( remove_targets_ );
	peopleTrackingArray_msg_.header.frame_id = "/map";

  ROS_INFO("PeopleSimulationAlgNode::init_sim:  antes scene_.read_destination_map2!!!");
  //read destinations
	if ( !scene_.read_destination_map2(  destination_map_path_.c_str() ) )
	{
		ROS_ERROR("PeopleSimulationAlgNode::init_sim(read_destination_map2): Could not read map destinations file !!!");
	}
  else{
		ROS_INFO("PeopleSimulationAlgNode::init_sim(read_destination_map2): read destinations map file : SUCCESS!!!");
	}
	if ( !scene_.get_akp_for_person_companion()->read_destination_map2(  destination_map_path_.c_str() ) )
	{
		ROS_ERROR("PeopleSimulationAlgNode::init_sim(read_destination_map2): Could not read map destinations file for akp plannin of companion person!!!");
	}
  else{
		ROS_INFO("PeopleSimulationAlgNode::init_sim(read_destination_map2): read destinations map file for akp plannin of companion person: SUCCESS!!!");
	}
ROS_INFO("PeopleSimulationAlgNode::init_sim:  despues scene_.read_destination_map2!!!");
  //read force map
	if ( !scene_.read_force_map(  force_map_path_.c_str() ) )
	{
		ROS_ERROR("PeopleSimulationAlgNode::init_sim: Could not read map force file !!!");
	}
  else{
		ROS_INFO("PeopleSimulationAlgNode::init_sim: read map force file : SUCCESS!!!");
	}

  if ( !scene_.get_akp_for_person_companion()->read_force_map(  force_map_path_.c_str() ) )
	{
		ROS_ERROR("PeopleSimulationAlgNode::init_sim: Could not read map force file for akp plannin of companion person !!!");
	}
  else{
		ROS_INFO("PeopleSimulationAlgNode::init_sim: read map force file for akp plannin of companion person: SUCCESS!!!");
	}
	//false = no destroy simulated persons
	//scene_.set_remove_targets(true);

	//target marker
	track_marker_.header.frame_id = "/map";
	track_marker_.ns = "tracks";
	track_marker_.type = visualization_msgs::Marker::CYLINDER;
	track_marker_.action = visualization_msgs::Marker::ADD;
	track_marker_.lifetime = ros::Duration(1.0f);
	track_marker_.scale.x = 0.5;
	track_marker_.scale.y = 0.5;
	track_marker_.scale.z = 0.6;
	track_marker_.color.a = 0.6;
	track_marker_.color.r = 0.0;
	track_marker_.color.g = 1.0;
	track_marker_.color.b = 0.0;
	track_marker_.pose.position.z = 0.3;

	id_marker_.header.frame_id = "/map";
	id_marker_.ns = "ids";
	id_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	id_marker_.action = visualization_msgs::Marker::ADD;
	id_marker_.lifetime = ros::Duration(1.0f);
	id_marker_.scale.z = 0.5;
	id_marker_.pose.position.z = 1.0;
	id_marker_.color.a = 0.5;
	id_marker_.color.r = 0.0;
	id_marker_.color.g = 0.0;
	id_marker_.color.b = 0.0;
	ROS_INFO("PeopleSimulationAlgNode::init_sim:  SUCCESS!!!");

  init_person_companion_markers();
 ROS_INFO("out PeopleSimulationAlgNode::init_sim()");
}

void PeopleSimulationAlgNode::mainNodeThread(void)
{

  if(debug_person_companion_){
    if(freeze_){
      ROS_INFO(" IN PeopleSimulationAlgNode::mainNodeThread; freeze_=true");
    }else{
      ROS_INFO(" IN PeopleSimulationAlgNode::mainNodeThread; freeze_=false");
    }
  } 

  // [fill msg structures]
  //peopleTrackingArray_msg_.header.seq = seq_cont_; //autocompeted by ros.... lol
  peopleTrackingArray_msg_.header.stamp = ros::Time::now();

  if(debug_person_companion_){
    ROS_INFO(" 1 IN PeopleSimulationAlgNode::mainNodeThread");
  }

  //for every new trajectory
	iri_perception_msgs::detection person;

   if(debug_person_companion_){
      ROS_INFO(" 2 IN PeopleSimulationAlgNode::mainNodeThread");
   }

  //update front and rear obstacles (ely-include obstacles in people simulation)
  this->laser_points.clear();

  this->fscan_mutex_enter2();
  if(this->fscan_received)
  {
     //   ROS_INFO("(Csim) fscan_received");
    this->laser_points = this->scan2points(this->fscan);
      //  ROS_INFO("sale de scan2points");
    this->fscan_received=false;

  }

  if(debug_person_companion_){
    ROS_INFO(" 3 IN PeopleSimulationAlgNode::mainNodeThread");
  }
  this->fscan_mutex_exit2();
  this->rscan_mutex_enter2();

  if(this->rscan_received)
  {
    //  ROS_INFO(" (Csim) rscan_received");
    std::vector< Spoint > rear_points;
    rear_points = this->scan2points(this->rscan);
    this->laser_points.insert(this->laser_points.end(), rear_points.begin(), rear_points.end());
    this->rscan_received=false;
  }

  if(debug_person_companion_){
    ROS_INFO(" 4 IN PeopleSimulationAlgNode::mainNodeThread");
  }
  this->rscan_mutex_exit2();
  this->scene_.read_laser_scan( this->laser_points, true );
 // this->scene_.get_akp_for_person_companion()->read_laser_scan( this->laser_points, true ); // incluir laser points en el planner_akp_for_person_companion

  // ROS_INFO("despues read scans");

  if(debug_person_companion_){
    ROS_INFO(" 5 IN PeopleSimulationAlgNode::mainNodeThread");
  }

	//scene update
    this->alg_.lock();
  //ROS_INFO("despues alg_.lock()");
  // ------------------------------------------------------------------------
  //Get Robot position: transform empty/zero pose from base_link to map
  std::string target_frame = "/map";
  //TODO set param in .launch  <param name="~/robot" type="string" value="$(env ROBOT)" />
  std::string source_frame = "/" + this->robot_ + "/base_link";  // Target frame del robot en simulacion!!!

  if(debug_person_companion_){
    ROS_INFO(" 6 IN PeopleSimulationAlgNode::mainNodeThread");
  }
  ros::Time target_time    = ros::Time::now();
  //ROS_INFO("target_frame.c_str()=%s",target_frame.c_str());
  //ROS_INFO("target_frame.c_str()=%s",source_frame.c_str());
  bool tf_exists = tf_listener_.waitForTransform(target_frame, source_frame, target_time, ros::Duration(1), ros::Duration(0.01));
  //ROS_INFO("PeopleSimulationAlgNode::mainNodeThread: antes if(tf_exists)"); 
  if(debug_person_companion_){ 
    ROS_INFO(" 7 IN PeopleSimulationAlgNode::mainNodeThread");
  }

  if(tf_exists)
  {
    geometry_msgs::PoseStamped poseIn;
    geometry_msgs::PoseStamped poseOut;
    poseIn.header.stamp    = target_time;
    poseIn.header.frame_id = source_frame;
    poseIn.pose.orientation.z = 1.0; //valid quaternion
    tf_listener_.transformPose(target_frame, poseIn, poseOut);
    //TODO add velocities in /map coordinates to the scene update
    Crobot* person_companion_in=scene_.get_akp_for_person_companion()->get_person_companion_akp();
    SpointV_cov person_Spoint_in=person_companion_in->get_current_pointV();
    person_Spoint_in.time_stamp=poseOut.header.stamp.toSec();
    Spose person_Spose_in=person_companion_in->get_current_pose();
    person_Spose_in.time_stamp=poseOut.header.stamp.toSec();
    scene_.update_robot(Spose(poseOut.pose.position.x, poseOut.pose.position.y, poseOut.header.stamp.toSec()));
    //scene_.update_person_companion(person_Spose_in,person_Spoint_in);
    scene_.get_akp_for_person_companion()->update_robot(Spose(poseOut.pose.position.x, poseOut.pose.position.y, poseOut.header.stamp.toSec()));
    //ROS_INFO("PeopleSimulationAlgNode::mainNodeThread: if(tf_exists)");  
  }
  else
  {
    ROS_INFO("fail, no transform found");
  }

  if(debug_person_companion_){
    ROS_INFO(" 8 IN PeopleSimulationAlgNode::mainNodeThread");
  }
  //ROS_INFO("PeopleSimulationAlgNode::mainNodeThread: despues if(tf_exists)");  
	std::vector<SdetectionObservation> obs_scene;
  // ROS_INFO("PeopleSimulationAlgNode::mainNodeThread: antes obs_scene.push_back");  
	obs_scene.push_back( SdetectionObservation(0, ros::Time::now().toSec() ));//void observation, just for the timestamp
  if(debug_person_companion_){
    ROS_INFO(" 9 IN PeopleSimulationAlgNode::mainNodeThread");
  }
	//scene_.update_scene( obs_scene );
  if(debug_person_companion_){
    ROS_INFO( "10 IN PeopleSimulationAlgNode::mainNodeThread; antes !!! scene_.update_scene_companion_simulation");
  }
  // Spose best_next_pose;
  scene_.update_scene_companion_simulation(obs_scene,ini_companion_); // OJO!!! modificado para hacer grupos de personas.
  //Spose pose_command; // solo para que concuerde con el simulation con person companion que usa el akp.
   // pose_command=Spose();
  //scene_.update_scene_companion_simulation_akp_person_companion(obs_scene,ini_companion_,ini_person_theta,pose_command); //modificado grupos + person companion akp
  new_person_companion_position_=this->scene_.get_person_companion_person_abstract();


  if(debug_person_companion_){
    ROS_INFO(" 11 IN PeopleSimulationAlgNode::mainNodeThread");
  }
 // Cperson_abstract::companion_reactive reactive_;
 // ROS_INFO( "antes !!! de get_akp_for_person_companion()->person_companion_plan_companion(best_next_pose,reactive_);");
 // this->get_akp_for_person_companion()->person_companion_plan_companion(best_next_pose,reactive_);
 // robot_pose_=best_next_pose;
//  ROS_INFO( "despues !!! scene_.update_scene_companion_simulation");
  ini_companion_=false; // despues de inicializar las personas lo pones a falso.
  obs.clear();
  //publish data
  const std::list<Cperson_abstract *>* person_list = scene_.get_scene( ); // Todo, puede que esta tenga que ser ahora la interna, no esta...
  const std::list<Cperson_abstract *>* person_list2 = scene_.get_akp_for_person_companion()->get_scene( );

  if(debug_person_companion_){
    ROS_INFO("2");
  }
  if( !freeze_ )
  {
    if(debug_person_companion_){
      if(person_list->empty()){
        ROS_INFO("in freze; person_list->empty()=true");
      }else{
        ROS_INFO("in freze; person_list->empty()=false");
      }
      if(person_list2->empty()){
        ROS_INFO("in freze; person_list2->empty()=true");
      }else{
        ROS_INFO("in freze; person_list2->empty()=false");
      }
    }
    peopleTrackingArray_msg_.detection.clear();
    for( std::list<Cperson_abstract*>::const_iterator iit = person_list->begin();
          iit!=person_list->end(); iit++ )
    {
		  person.id = (*iit)->get_id();
		  person.position.x = (*iit)->get_current_pointV().x;
		  person.position.y = (*iit)->get_current_pointV().y;
		  person.velocity.x = (*iit)->get_current_pointV().vx;
		  person.velocity.y = (*iit)->get_current_pointV().vy;
      //covariances conversion from 4x4 to 6x6
    	person.covariances[0] = (*iit)->get_current_pointV().cov[0];//x x
    	person.covariances[1] = (*iit)->get_current_pointV().cov[1];//x y
    	person.covariances[3] = (*iit)->get_current_pointV().cov[2];//x vx
    	person.covariances[4] = (*iit)->get_current_pointV().cov[3];//x vy
    	person.covariances[6] = (*iit)->get_current_pointV().cov[4];//x y
    	person.covariances[7] = (*iit)->get_current_pointV().cov[5];//y y
    	person.covariances[9] = (*iit)->get_current_pointV().cov[6];//y vx
    	person.covariances[10] = (*iit)->get_current_pointV().cov[7];//y vy
    	person.covariances[18] = (*iit)->get_current_pointV().cov[8];//vx x
    	person.covariances[19] = (*iit)->get_current_pointV().cov[9];//vx y
    	person.covariances[21] = (*iit)->get_current_pointV().cov[10];//vx vx
    	person.covariances[22] = (*iit)->get_current_pointV().cov[11];//vx vy
    	person.covariances[24] = (*iit)->get_current_pointV().cov[12];//vy x
    	person.covariances[25] = (*iit)->get_current_pointV().cov[13];//vy y
    	person.covariances[27] = (*iit)->get_current_pointV().cov[14];//vy vx
    	person.covariances[28] = (*iit)->get_current_pointV().cov[15];//vy vy
		  peopleTrackingArray_msg_.detection.push_back(person);

     // (*iit)->print();

      std::vector<double> cov;
      cov.resize(16,0.0);
      cov[0] = (*iit)->get_current_pointV().cov[0];//x x
      cov[1] = (*iit)->get_current_pointV().cov[1];//x y
      cov[2] = (*iit)->get_current_pointV().cov[2];//x vx
      cov[3] = (*iit)->get_current_pointV().cov[3];//x vy
      cov[4] = (*iit)->get_current_pointV().cov[4];//x y
      cov[5] = (*iit)->get_current_pointV().cov[5];//y y
      cov[6] = (*iit)->get_current_pointV().cov[6];//y vx
      cov[7] = (*iit)->get_current_pointV().cov[7];//y vy
      cov[8] = (*iit)->get_current_pointV().cov[8];//vx x
      cov[9] = (*iit)->get_current_pointV().cov[9];//vx y
      cov[10] = (*iit)->get_current_pointV().cov[10];//vx vx
      cov[11] = (*iit)->get_current_pointV().cov[11];//vx vy
      cov[12] = (*iit)->get_current_pointV().cov[12];//vy x
      cov[13] = (*iit)->get_current_pointV().cov[13];//vy y
      cov[14] = (*iit)->get_current_pointV().cov[14];//vy vx
      cov[15] = (*iit)->get_current_pointV().cov[15];//vy vy
      double x=(*iit)->get_current_pointV().x;
      double y=(*iit)->get_current_pointV().y;
      double vx=(*iit)->get_current_pointV().vx;
      double vy=(*iit)->get_current_pointV().vy;
      int id=(*iit)->get_id();
      obs.push_back(SdetectionObservation(id, ros::Time::now().toSec(),x, y ,vx, vy, cov));
	  }

    /*  person companion track (for publish) */

    person.id = scene_.get_id_person_companion();
	  person.position.x = new_person_companion_position_.get_current_pointV().x;
	  person.position.y = new_person_companion_position_.get_current_pointV().y;
	  person.velocity.x = new_person_companion_position_.get_current_pointV().vx;
	  person.velocity.y = new_person_companion_position_.get_current_pointV().vy;
    //covariances conversion from 4x4 to 6x6
    person.covariances[0] = new_person_companion_position_.get_current_pointV().cov[0];//x x
    person.covariances[1] = new_person_companion_position_.get_current_pointV().cov[1];//x y
    person.covariances[3] = new_person_companion_position_.get_current_pointV().cov[2];//x vx
    person.covariances[4] = new_person_companion_position_.get_current_pointV().cov[3];//x vy
    person.covariances[6] = new_person_companion_position_.get_current_pointV().cov[4];//x y
    person.covariances[7] = new_person_companion_position_.get_current_pointV().cov[5];//y y
    person.covariances[9] = new_person_companion_position_.get_current_pointV().cov[6];//y vx
    person.covariances[10] = new_person_companion_position_.get_current_pointV().cov[7];//y vy
    person.covariances[18] = new_person_companion_position_.get_current_pointV().cov[8];//vx x
    person.covariances[19] = new_person_companion_position_.get_current_pointV().cov[9];//vx y
    person.covariances[21] = new_person_companion_position_.get_current_pointV().cov[10];//vx vx
    person.covariances[22] = new_person_companion_position_.get_current_pointV().cov[11];//vx vy
    person.covariances[24] = new_person_companion_position_.get_current_pointV().cov[12];//vy x
    person.covariances[25] = new_person_companion_position_.get_current_pointV().cov[13];//vy y
    person.covariances[27] = new_person_companion_position_.get_current_pointV().cov[14];//vy vx
    person.covariances[28] = new_person_companion_position_.get_current_pointV().cov[15];//vy vy
		peopleTrackingArray_msg_.detection.push_back(person);


    if(debug_person_companion_){
      std::cout << "(nodo) std::vector<SdetectionObservation> obs:"<< std::endl;
	    for( unsigned int ty = 0 ; ty< this->obs.size() ; ty++)
	    {
			  obs[ty].print();
	    }
    }
   // bool have_companion_person=true;
	 // scene_.get_akp_for_person_companion()->update_scene(obs,have_companion_person, true);

    if(debug_person_companion_){
      ROS_INFO(" 11.2 (print1) in");
      scene_.get_akp_for_person_companion()->print();
    }
    
  }
  //ROS_INFO(" 11.3 (print2) out");
  //scene_.get_akp_for_person_companion()->print2();
  
  if(debug_person_companion_){
    ROS_INFO(" 12 IN PeopleSimulationAlgNode::mainNodeThread");
  }

	if( vis_mode_ && !freeze_)
	{
		MarkerArray_msg_.markers.clear();
		track_marker_.header.stamp =   peopleTrackingArray_msg_.header.stamp;
		id_marker_.header.stamp =   peopleTrackingArray_msg_.header.stamp;
    // draw targets
    for( std::list<Cperson_abstract*>::const_iterator iit = person_list->begin();
          iit!=person_list->end(); iit++ )
    {
			track_marker_.id = (*iit)->get_id();
			track_marker_.pose.position.x = (*iit)->get_current_pointV().x;
			track_marker_.pose.position.y = (*iit)->get_current_pointV().y;

     // ROS_INFO(" PeopleSimulationAlgNode::mainNodeThread; track_marker_=%d; track_marker_.pose.position.x=%f; track_marker_.pose.position.y=%f",track_marker_.id,track_marker_.pose.position.x,track_marker_.pose.position.y);

			MarkerArray_msg_.markers.push_back(track_marker_);
			id_marker_.id = (*iit)->get_id();
			id_marker_.pose.position.x = (*iit)->get_current_pointV().x;
			id_marker_.pose.position.y = (*iit)->get_current_pointV().y;
			std::ostringstream target_id;
			target_id << id_marker_.id;
			id_marker_.text = target_id.str();
			MarkerArray_msg_.markers.push_back(id_marker_);
		}

    /* Person companion marker (track) */
      track_marker_.id = scene_.get_id_person_companion(); // get_companion id...
			track_marker_.pose.position.x =new_person_companion_position_.get_current_pointV().x;
			track_marker_.pose.position.y = new_person_companion_position_.get_current_pointV().y;

     // ROS_INFO(" PeopleSimulationAlgNode::mainNodeThread; track_marker_=%d; track_marker_.pose.position.x=%f; track_marker_.pose.position.y=%f",track_marker_.id,track_marker_.pose.position.x,track_marker_.pose.position.y);

			MarkerArray_msg_.markers.push_back(track_marker_);
			id_marker_.id = scene_.get_id_person_companion(); // get_companion id...
			id_marker_.pose.position.x = new_person_companion_position_.get_current_pointV().x;
			id_marker_.pose.position.y = new_person_companion_position_.get_current_pointV().y;
			std::ostringstream target_id;
			target_id << id_marker_.id;
			id_marker_.text = target_id.str();
			MarkerArray_msg_.markers.push_back(id_marker_);

    //draw robot if any  (cambio ely, NO quiero un marker verde en el robot, mientras simulo el akp!!!!)
   /* if (  scene_.get_robot() != NULL )
    {
		  track_marker_.id = 0;
    	track_marker_.ns = "robot";
		  track_marker_.pose.position.x = scene_.get_robot()->get_current_pointV().x;
		  track_marker_.pose.position.y = scene_.get_robot()->get_current_pointV().y;
		  MarkerArray_msg_.markers.push_back(track_marker_);
    	track_marker_.ns = "tracks";
    }*/
    if(debug_person_companion_){
      ROS_INFO(" 13 IN PeopleSimulationAlgNode::mainNodeThread");
    }
		this->tracksMarkers_publisher_.publish(this->MarkerArray_msg_);

	}
  
   // slice_plan( );
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
// ROS_INFO(" FIN PeopleSimulationAlgNode::mainNodeThread");

  personCompanion_MarkerArray_msg_.markers.clear();
  init_person_companion_markers();
 
  fill_scene_markers();
  fill_best_path_2d();
 

  if( vis_mode2_ > 1 ) //normal mode 2d features
  {
    fill_planning_markers_2d();  
    //fill_people_prediction_markers_2d();
    //fill_forces_markers();
    fill_laser_obstacles();
  }
  if ( vis_mode2_ > 2 )//super mode 3d features
  {
  //fill_people_prediction_markers_3d();
  //fill_planning_markers_3d();
  }
  
  
  this->personCompanionAkpMarkers_publisher_.publish(this->personCompanion_MarkerArray_msg_);
  this->tracks_publisher_.publish(this->peopleTrackingArray_msg_);
  if(debug_person_companion_){
    ROS_INFO(" 14 FIN IN PeopleSimulationAlgNode::mainNodeThread");
  }

  this->alg_.unlock();
}

/*  [subscriber callbacks] */
void PeopleSimulationAlgNode::reset_mutex_enter(void){
  pthread_mutex_lock(&this->reset_mutex_);
}
void PeopleSimulationAlgNode::reset_mutex_exit(void){
  pthread_mutex_unlock(&this->reset_mutex_);
}

/*  [subscriber callbacks] (fake laser)*/
void PeopleSimulationAlgNode::fscan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
  //ROS_INFO("PeopleSimulationAlgNode::fscan_callback: New Message Received"); 

  this->fscan_mutex_enter2(); 
  this->fscan = *msg;
  this->fscan_received=true;
  this->fscan_mutex_exit2(); 
}

void PeopleSimulationAlgNode::rscan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
 // ROS_INFO("PeopleSimulationAlgNode::rscan_callback: New Message Received"); 

  this->rscan_mutex_enter2(); 
  this->rscan = *msg;
  this->rscan_received=true;
  this->rscan_mutex_exit2(); 
}

/*  [service callbacks] */
bool PeopleSimulationAlgNode::resetCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) 
{ 
  ROS_INFO("PeopleSimulationAlgNode::resetCallback: New Request Received!"); 

  //use appropiate mutex to shared variables if necessary 
  //this->reset_mutex_enter(); 

  if( simulation_mode_ == PeopleSimulationAlgNode::Density_incremental )
  {
    //TODO: set the incremental proportion
    //n_persons_ = (num_experiment_/25 + 1)*25;
    //scene_.set_number_virtual_people( n_persons_ );
  }

  
  this->alg_.lock(); 
  ROS_INFO("PeopleSimulationAlgNode::resetCallback: Processin New Request!"); 
  //do operations with req and output on res 
  scene_.clear_scene();
  scene_.set_number_virtual_people( n_persons_ );
  this->alg_.unlock(); 

  //unlock previously blocked shared variables 
  //this->reset_mutex_exit(); 

  return true; 
}

/*  [action callbacks] */

/*  [action requests] */

void PeopleSimulationAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

	ROS_INFO("         *******  algorithm config update  *******\n\n");
  simulation_mode_ = (PeopleSimulationAlgNode::simulation_mode) config.simulation_mode;
  vis_mode_ = config.vis_mode;
	n_persons_ = config.number_persons;
	freeze_ = config.freeze;
	scene_.set_number_virtual_people( n_persons_ );
  scene_.set_robot_person_proximity_distance(config.proximity_distance_between_robot_and_person);
  scene_.set_proximity_distance_tolerance(config.proximity_distance_tolerance);
  scene_.set_id_person_companion(config.id_person_companion);
    this->person_companion_goal_force_marker       = config.person_companion_goal_force_marker;
    this->person_companion_person_forces_marker    = config.person_companion_person_forces_marker;
    this->person_companion_obstacles_forces_marker = config.person_companion_obstacles_forces_marker;
    this->person_companion_resultant_force_marker  = config.person_companion_resultant_force_marker;
ROS_INFO(" 1 algorithm config update \n");
    person_companion_goal_providing_mode_ = (PeopleSimulationAlgNode::Goal_providing_mode)config.person_companion_goal_providing_mode;
ROS_INFO(" 2 algorithm config update \n");
    person_companion_slicing_path_diff_orientation_ = config.person_companion_slicing_diff_orientation;
    ini_person_theta=config.ini_theta_companion_person;
ROS_INFO(" 3 algorithm config update \n");
this->vis_mode2_ = config.vis_mode2;

  this->alg_.unlock();
}

void PeopleSimulationAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<PeopleSimulationAlgNode>(argc, argv, "people_simulation_alg_node");
}



std::vector<Spoint> PeopleSimulationAlgNode::scan2points(const sensor_msgs::LaserScan scan)
{
  
  //header_point_.header= scan.header;  // variable for companion (ely)

  std::vector<Spoint> points;
  std::string source_frame = scan.header.frame_id;
  std::string target_frame = this->fixed_frame;
  ros::Time target_time    = scan.header.stamp;
  //ROS_INFO("PeopleSimulationAlgNode::scan2points: 1");
if(source_frame!=target_frame)
{
  try
  {
  //ROS_INFO("PeopleSimulationAlgNode::scan2points: 2 target_time=%f",target_time.toSec());
    bool tf_exists = this->tf_listener2_->waitForTransform(target_frame, source_frame, target_time, ros::Duration(1), ros::Duration(0.01));
  //ROS_INFO("PeopleSimulationAlgNode::scan2points: 3");
    if(tf_exists)
    {
//ROS_INFO("PeopleSimulationAlgNode::scan2points: 4");


    geometry_msgs::PoseStamped poseIn;
    geometry_msgs::PoseStamped poseOut;
    poseIn.header = scan.header;
    poseIn.pose.orientation.z = 1.0; //valid quaternion

     // geometry_msgs::PointStamped pointIn;
     // geometry_msgs::PointStamped pointOut;
      //pointIn.header = scan.header;
//ROS_INFO("PeopleSimulationAlgNode::scan2points: 5");
      double t;
      for( unsigned int i = 0; i < scan.ranges.size(); ++i)
      {
//ROS_INFO("PeopleSimulationAlgNode::scan2points: 6");
        if ( scan.ranges[i]  < 15.0  && scan.ranges[i] > 0.05)
        {
//ROS_INFO("PeopleSimulationAlgNode::scan2points: 7");
          t = scan.angle_min + (double)i*scan.angle_increment;
//ROS_INFO("PeopleSimulationAlgNode::scan2points: 8");
          //pointIn.point.x = scan.ranges[i] * cos( t );
          poseIn.pose.position.x = scan.ranges[i] * cos( t );
//ROS_INFO("PeopleSimulationAlgNode::scan2points: 9");
          //pointIn.point.y = scan.ranges[i] * sin( t );
          poseIn.pose.position.y = scan.ranges[i] * sin( t );
/*ROS_INFO("PeopleSimulationAlgNode::scan2points: 10");
ROS_INFO("PeopleSimulationAlgNode::scan2points: %s",source_frame.c_str());
ROS_INFO("PeopleSimulationAlgNode::scan2points: %s",target_frame.c_str());
ROS_INFO("PeopleSimulationAlgNode::scan2points: %f",poseIn.pose.position.x);
ROS_INFO("PeopleSimulationAlgNode::scan2points: %f",poseIn.pose.position.y);
ROS_INFO("PeopleSimulationAlgNode::scan2points: %f",poseOut.pose.position.x);
ROS_INFO("PeopleSimulationAlgNode::scan2points: %f",poseOut.pose.position.y);
*/
          this->tf_listener2_->transformPose(target_frame, poseIn, poseOut);
//ROS_INFO("PeopleSimulationAlgNode::scan2points: 11");
          points.push_back( Spoint( poseOut.pose.position.x, poseOut.pose.position.y ) );
//ROS_INFO("PeopleSimulationAlgNode::scan2points: 12");
        }
      }
    }
    else
    {
      ROS_WARN("PeopleSimulationAlgNode::scan2points: No transform: %s-->%s", source_frame.c_str(), target_frame.c_str());
    }
  }
  catch(tf::TransformException &ex)
  {
    ROS_WARN("PeopleSimulationAlgNode::scan2points: %s",ex.what());
  }
}else{
  //ROS_INFO("entro en same frame id");
  double t;
  for( unsigned int i = 0; i < scan.ranges.size(); ++i)
  {
    if ( scan.ranges[i]  < 15.0  && scan.ranges[i] > 0.05)
    {
      t = scan.angle_min + (double)i*scan.angle_increment;
      //ROS_INFO("point.x=%f",scan.ranges[i] * cos( t ));
      //ROS_INFO("point.y=%f",scan.ranges[i] * sin( t ));
      points.push_back( Spoint(  scan.ranges[i] * cos( t ), scan.ranges[i] * sin( t ) ) );
    }

  }
}
//ROS_INFO("PeopleSimulationAlgNode::scan2points: 13");
  return points;
}

bool PeopleSimulationAlgNode::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  ROS_INFO("!!!!!!!!!!!!!!!!!!! ENTRO EN PeopleSimulationAlgNode::setPlan!!!");
  // (1) funcion, local planer. Da las posiciones del local plan, path, a seguir por el robot. la que toca de verdad la libreria de Gonzalo. El plan que el calcula.
  // ROS_INFO("PeopleSimulationAlgNode::setPlan");
  if(!initialized_)
  {
    ROS_ERROR("PeopleSimulationAlgNode::setPlan: This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }
 // velocities_.clear();//clears the average of velocities when new plans are calculated
  this->global_plan_.clear();
  // robot_goal=scene_.get_akp_for_person_companion()->get_robot_goal(); // (ely) puesto para petarme el goal externo del rviz!!!
  //orig_global_plan.pose.position.x=robot_goal.x;
  // orig_global_plan.pose.position.y=robot_goal.y;
  this->global_plan_ =  orig_global_plan;
  // if(debug_antes_subgoals_entre_AKP_goals_){
  ROS_INFO("PeopleSimulationAlgNode::setPlan: global_plan_ points.x= %f ; points.y= %f", orig_global_plan.back().pose.position.x, orig_global_plan.back().pose.position.y);
  ROS_INFO("PeopleSimulationAlgNode::setPlan: global_plan_ points %d", uint(this->global_plan_.size()));
  //}
  // ROS_INFO("PeopleSimulationAlgNode::setPlan: global_plan_ points %d", uint(this->global_plan_.size()));
  // slice global plan into subgoals
  if ( person_companion_goal_providing_mode_ == PeopleSimulationAlgNode::Slicing_global )
    slice_plan( );

  //else , there is no need to do anything
  return true;
}

void PeopleSimulationAlgNode::slice_plan()
{

  //ROS_INFO("ENTRO EN PeopleSimulationAlgNode::slice_plan!!!");

  sliced_global_plan_.clear();
  std::vector<double> orientations, avg_orientations;
  unsigned int n(5), i_down, i_up;
  double dx, dy, theta, accumulated_orientation, avg_ori;
  
  // get vector of orientaions in 2D-> yaw angle
  assert(orientations.size() <=1 && "PeopleSimulationAlgNode::slice_plan: global_plan vector empty");
  for( unsigned int i=1; i < global_plan_.size(); ++i )
  {
    /*fake orientation message, filled with constant q=[0,0,0,1] o.O
    ROS_INFO( "quaternion entering = (%f, %f, %f, %f)" ,global_plan_[i].pose.orientation.x,
                      global_plan_[i].pose.orientation.y,
                      global_plan_[i].pose.orientation.z,
                      global_plan_[i].pose.orientation.w );*/
    
    //differential in 2d
    dy = global_plan_[i].pose.position.y - global_plan_[i-1].pose.position.y;
    dx = global_plan_[i].pose.position.x - global_plan_[i-1].pose.position.x;
    orientations.push_back( atan2( dy, dx ) );
    //ROS_INFO("2d processing = %f", theta);
  }
  
  
  assert(!orientations.empty() && "PeopleSimulationAlgNode::slice_plan: orientations vector empty");
  //calculate the differential of orientations in 2D in [-pi, pi]
  for(unsigned int i=1; i < orientations.size(); ++i)
  {
    orientations[i-1] = orientations[i] - orientations[i-1];
    if( orientations[i-1] > PI )
      orientations[i-1] -= 2.0*PI;
    if (orientations[i-1] < -PI )
      orientations[i-1] += 2.0*PI;
  }
  orientations.back() = 0.0;
  
  //smooth variantions of orientations and absolute value [0,pi]
  /*
  for(unsigned int i=0; i < orientations.size(); ++i)
  {
    if( i > n ) i_down = i-n;
    else i_down = 0;
    if ( i + n < orientations.size() ) i_up = i+n;
    else i_up = orientations.size();
    avg_ori = 0.0;
    for ( unsigned int j = i_down; j < i_up; ++j )
    {
      avg_ori += orientations[j];
    }
    avg_orientations.push_back( fabs(avg_ori/(double) n) );
    
  } not necessary now*/
  
  //slice the std::vector<geometry_msgs::PoseStamped> global_plan_
  //ROS_INFO("accumulated orientation = %d", orientations.size());
  //initial pose is the last of the plan TODO esto va mal, peta y se pasa del i
  sliced_global_plan_.push_back( global_plan_.back() );
  accumulated_orientation = 0.0;
  for(unsigned int i=orientations.size()-1; i > 0 ; --i)
  {
    accumulated_orientation += fabs(orientations[i]);
    //ROS_INFO("accumulated orientation = %f with respect to %f in %d" , accumulated_orientation, 20.0*PI/180.0, i);
    if ( accumulated_orientation > person_companion_slicing_path_diff_orientation_*PI/180.0 )
    {
      //check if it near the previous on and then discard
      dx = sliced_global_plan_.back().pose.position.x - global_plan_[i].pose.position.x;
      dy = sliced_global_plan_.back().pose.position.y - global_plan_[i].pose.position.y;
      accumulated_orientation = 0.0;
      if ( dx*dx + dy*dy > 1.0 )
      {
        sliced_global_plan_.push_back( global_plan_[i] );
        //ROS_INFO( "Subgoal in (%f, %f) at i = %d", sliced_global_plan_.back().pose.position.x, sliced_global_plan_.back().pose.position.y, i );
      }
    }  
  }
  
  //for( unsigned int i=0; i < sliced_global_plan_.size(); ++i )
  //  ROS_INFO( "PeopleSimulationAlgNode::slice_plan: Subgoal %d in (%f, %f)", i, sliced_global_plan_[i].pose.position.x, sliced_global_plan_[i].pose.position.y );
      
}



void PeopleSimulationAlgNode::init_person_companion_markers()
{

 // ROS_INFO( " 1 init_person_companion_markers() ");
  //target marker. Predicts the trajectories of people
  personCompanion_pred_traj_marker_.header.frame_id = this->fixed_frame;
  personCompanion_pred_traj_marker_.ns = "pred";
  personCompanion_pred_traj_marker_.type = visualization_msgs::Marker::CYLINDER;
  personCompanion_pred_traj_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_pred_traj_marker_.lifetime = ros::Duration(1.23f);
 // ROS_INFO( " 1 init_person_companion_markers(), %f ", scene_.get_akp_for_person_companion()->get_dt());
  personCompanion_pred_traj_marker_.scale.z = scene_.get_akp_for_person_companion()->get_dt();
 // ROS_INFO( " 1.2 init_person_companion_markers(), %f ", scene_.get_akp_for_person_companion()->get_dt());
  personCompanion_pred_traj_marker_.color.a = 0.05;
  personCompanion_pred_traj_marker_.color.r = 0.3;
  personCompanion_pred_traj_marker_.color.g = 1.0;
  personCompanion_pred_traj_marker_.color.b = 0.3;
  personCompanion_pred_traj_marker_.pose.orientation.w = 1.0;
  // ROS_INFO( " 2 init_person_companion_markers() ");
  //target marker of the best path, projected in 2d
  personCompanion_pred_traj2d_marker_.header.frame_id = this->fixed_frame;
  personCompanion_pred_traj2d_marker_.ns = "pred2d";
  personCompanion_pred_traj2d_marker_.type = visualization_msgs::Marker::CYLINDER;
  personCompanion_pred_traj2d_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_pred_traj2d_marker_.lifetime = ros::Duration(1.23f);
  personCompanion_pred_traj2d_marker_.scale.z = 0.05;
  personCompanion_pred_traj_marker_.color.a = 0.4;
  personCompanion_pred_traj_marker_.color.r = 0.3;
  personCompanion_pred_traj_marker_.color.g = 1.0;
  personCompanion_pred_traj_marker_.color.b = 0.3;
  personCompanion_pred_traj_marker_.pose.orientation.w = 1.0;
  personCompanion_pred_traj2d_marker_.pose.position.z = 0.025;
// ROS_INFO( " 3 init_person_companion_markers() ");
  //cylinder marker, for destinations and goals
  personCompanion_cylinder_marker_.header.frame_id = this->fixed_frame;
  personCompanion_cylinder_marker_.ns = "scene";
  personCompanion_cylinder_marker_.type = visualization_msgs::Marker::CYLINDER;
  personCompanion_cylinder_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_cylinder_marker_.lifetime = ros::Duration(1.3f);
  personCompanion_cylinder_marker_.color.a = 1.0;
  personCompanion_cylinder_marker_.color.r = 0.3;
  personCompanion_cylinder_marker_.color.g = 0.1;
  personCompanion_cylinder_marker_.color.b = 0.8;
  personCompanion_cylinder_marker_.scale.x = 0.7;
  personCompanion_cylinder_marker_.scale.y = 0.7;
  personCompanion_cylinder_marker_.scale.z = 1.2;
  personCompanion_cylinder_marker_.pose.position.z = 0.6;
  personCompanion_cylinder_marker_.pose.orientation.w = 1.0;
// ROS_INFO( " 4 init_person_companion_markers() ");
  // robot goal marker, cylinders plot in the scene
  personCompanion_robot_goal_marker_.header.frame_id = this->fixed_frame;
  personCompanion_robot_goal_marker_.ns = "scene";
  personCompanion_robot_goal_marker_.type = visualization_msgs::Marker::CYLINDER;
  personCompanion_robot_goal_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_robot_goal_marker_.lifetime = ros::Duration(1.3f);
  personCompanion_robot_goal_marker_.color.a = 1.0;
  personCompanion_robot_goal_marker_.color.r = 0.0; // 0.9
  personCompanion_robot_goal_marker_.color.g = 0.0; // 0.3
  personCompanion_robot_goal_marker_.color.b = 0.0; // 0.3
  personCompanion_robot_goal_marker_.scale.x = 0.4;
  personCompanion_robot_goal_marker_.scale.y = 0.4;
  personCompanion_robot_goal_marker_.scale.z = 0.2;
  personCompanion_robot_goal_marker_.pose.position.z = 0.1;
  personCompanion_robot_goal_marker_.pose.orientation.w = 1.0;
  
  personCompanion_robot_subgoal_marker_ = personCompanion_robot_goal_marker_;
  personCompanion_robot_subgoal_marker_.color.r = 0.9;
  personCompanion_robot_subgoal_marker_.color.g = 0.6;
  personCompanion_robot_subgoal_marker_.color.b = 0.6;
  personCompanion_robot_subgoal_marker_.scale.x = 0.3;
  personCompanion_robot_subgoal_marker_.scale.y = 0.3;

 //ROS_INFO( " 5 init_person_companion_markers() ");
  // robot_real_goal_marker!!!

  personCompanion_robot_goal_marker2_.header.frame_id = this->fixed_frame;
  personCompanion_robot_goal_marker2_.ns = "scene";
  personCompanion_robot_goal_marker2_.type = visualization_msgs::Marker::CYLINDER;
  personCompanion_robot_goal_marker2_.action = visualization_msgs::Marker::ADD;
  personCompanion_robot_goal_marker2_.lifetime = ros::Duration(1.3f);
  personCompanion_robot_goal_marker2_.color.a = 1.0;
  personCompanion_robot_goal_marker2_.color.r = 0.0;
  personCompanion_robot_goal_marker2_.color.g = 0.0;
  personCompanion_robot_goal_marker2_.color.b = 1.0;
  personCompanion_robot_goal_marker2_.scale.x = 0.4;
  personCompanion_robot_goal_marker2_.scale.y = 0.4;
  personCompanion_robot_goal_marker2_.scale.z = 1.0;
  personCompanion_robot_goal_marker2_.pose.position.z = 0.1;
  personCompanion_robot_goal_marker2_.pose.orientation.w = 1.0;

 //ROS_INFO( " 6 init_person_companion_markers() ");
  // robot_real_goal_marker!!!

  personCompanion_robot_goal_marker3_.header.frame_id = this->fixed_frame;
  personCompanion_robot_goal_marker3_.ns = "scene";
  personCompanion_robot_goal_marker3_.type = visualization_msgs::Marker::CYLINDER;
  personCompanion_robot_goal_marker3_.action = visualization_msgs::Marker::ADD;
  personCompanion_robot_goal_marker3_.lifetime = ros::Duration(1.3f);
  personCompanion_robot_goal_marker3_.color.a = 0.0;
  personCompanion_robot_goal_marker3_.color.r = 0.5;
  personCompanion_robot_goal_marker3_.color.g = 1.0;
  personCompanion_robot_goal_marker3_.color.b = 1.0;
  personCompanion_robot_goal_marker3_.scale.x = 1.4;
  personCompanion_robot_goal_marker3_.scale.y = 1.4;
  personCompanion_robot_goal_marker3_.scale.z = 2.0;
  personCompanion_robot_goal_marker3_.pose.position.z = 0.1;
  personCompanion_robot_goal_marker3_.pose.orientation.w = 1.0;

 //ROS_INFO( " 7 init_person_companion_markers() ");
  // workspace marker
  personCompanion_workspace_marker_.header.frame_id = this->fixed_frame;
  personCompanion_workspace_marker_.ns = "scene";
  personCompanion_workspace_marker_.type = visualization_msgs::Marker::LINE_LIST;
  personCompanion_workspace_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_workspace_marker_.lifetime = ros::Duration(1.3f);
  personCompanion_workspace_marker_.scale.x = 0.05;
  personCompanion_workspace_marker_.color.a = 0.8;
 //ROS_INFO( " 8 init_person_companion_markers() ");
  //planning marker. Calculated robot trajectories in space x time coordinates
  personCompanion_planning_marker_.header.frame_id = this->fixed_frame;
  personCompanion_planning_marker_.ns = "p3";
  personCompanion_planning_marker_.type = visualization_msgs::Marker::LINE_LIST;
  personCompanion_planning_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_planning_marker_.lifetime = ros::Duration(1.3f);
  personCompanion_planning_marker_.id = 0;
  personCompanion_planning_marker_.scale.x = 0.05;
  personCompanion_planning_marker_.color.a = 0.3;
  personCompanion_planning_marker_.color.r = 0.1;
  personCompanion_planning_marker_.color.g = 0.2;
  personCompanion_planning_marker_.color.b = 0.9;
 // ROS_INFO( " 8.1 init_person_companion_markers(), =%d, horitzon_time=%f; ", scene_.get_akp_for_person_companion()->get_number_of_vertex(  ), scene_.get_akp_for_person_companion()->get_horizon_time());
  //unsigned int num_vertex=scene_.get_akp_for_person_companion()->get_number_of_vertex(  );

  //personCompanion_planning_marker_.points.reserve( num_vertex ); // a pelo! mirar de arreglar, pq peta sin sentido...

 //ROS_INFO( " 9 init_person_companion_markers() ");
  //cylinder marker, planning random goals
  personCompanion_planning_goals_marker_.header.frame_id = this->fixed_frame;
  personCompanion_planning_goals_marker_.ns = "goals";
  personCompanion_planning_goals_marker_.type = visualization_msgs::Marker::CYLINDER;
  personCompanion_planning_goals_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_planning_goals_marker_.lifetime = ros::Duration(1.3f);
  personCompanion_planning_goals_marker_.color.a = 0.6;
  personCompanion_planning_goals_marker_.color.r = 0.9;
  personCompanion_planning_goals_marker_.color.g = 0.1;
  personCompanion_planning_goals_marker_.color.b = 0.0;
  personCompanion_planning_goals_marker_.scale.x = 0.2;
  personCompanion_planning_goals_marker_.scale.y = 0.2;
  personCompanion_planning_goals_marker_.scale.z = 0.1;
  personCompanion_planning_goals_marker_.pose.position.z = 0.05;
  personCompanion_planning_goals_marker_.pose.orientation.w = 1.0;

	// ROS_INFO( " 10 init_person_companion_markers() ");
  //best path marker
  personCompanion_best_path_marker_.header.frame_id = this->fixed_frame;
  personCompanion_best_path_marker_.ns = "best3";
  personCompanion_best_path_marker_.type = visualization_msgs::Marker::LINE_STRIP;
  personCompanion_best_path_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_best_path_marker_.lifetime = ros::Duration(1.3f);
  personCompanion_best_path_marker_.scale.x = 0.05;
  personCompanion_best_path_marker_.color.a = 0.7;
  personCompanion_best_path_marker_.color.r = 0.9;
  personCompanion_best_path_marker_.color.g = 0.2;
  personCompanion_best_path_marker_.color.b = 0.2;
	
  //best path marker
  personCompanion_best_path2d_marker_ = personCompanion_best_path_marker_;
  personCompanion_best_path2d_marker_.ns = "best2";
  personCompanion_best_path2d_marker_.id = 0;
 //ROS_INFO( " 11 init_person_companion_markers() ");
  //non-dominated marker
  personCompanion_nd_path_marker_.header.frame_id = this->fixed_frame;
  personCompanion_nd_path_marker_.ns = "nd3";
  personCompanion_nd_path_marker_.type = visualization_msgs::Marker::LINE_LIST;
  personCompanion_nd_path_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_nd_path_marker_.lifetime = ros::Duration(1.3f);
  personCompanion_nd_path_marker_.id = 0;
  personCompanion_nd_path_marker_.scale.x = 0.05;
  personCompanion_nd_path_marker_.color.a = 0.7;
  personCompanion_nd_path_marker_.color.r = 1.0;
  personCompanion_nd_path_marker_.color.g = 0.5;
  personCompanion_nd_path_marker_.color.b = 0.0;
	
  //non dominated path marker
  personCompanion_nd_path2d_marker_ = personCompanion_nd_path_marker_;
  personCompanion_nd_path2d_marker_.ns = "nd2";
  personCompanion_nd_path2d_marker_.id = 0;
 //ROS_INFO( " 12 init_person_companion_markers() ");
  //laser obstacle marker
  personCompanion_laser_obstacle_marker_.header.frame_id = this->fixed_frame;
  personCompanion_laser_obstacle_marker_.ns = "obstacles";
  personCompanion_laser_obstacle_marker_.type = visualization_msgs::Marker::CYLINDER;
  personCompanion_laser_obstacle_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_laser_obstacle_marker_.lifetime = ros::Duration(1.3f);
  personCompanion_laser_obstacle_marker_.color.a = 0.7;
  personCompanion_laser_obstacle_marker_.color.r = 0.0;
  personCompanion_laser_obstacle_marker_.color.g = 0.0;
  personCompanion_laser_obstacle_marker_.color.b = 0.0;
  personCompanion_laser_obstacle_marker_.scale.x = 0.5;
  personCompanion_laser_obstacle_marker_.scale.y = 0.5;
  personCompanion_laser_obstacle_marker_.scale.z = 0.2;
  personCompanion_laser_obstacle_marker_.pose.position.z = 0.1;
  personCompanion_laser_obstacle_marker_.pose.orientation.w = 1.0;
 //ROS_INFO( " 13 init_person_companion_markers() ");
  //force marker, a summation of all forces (red)
  personCompanion_force_marker_.header.frame_id = this->fixed_frame;
  personCompanion_force_marker_.ns =  "forces";
  personCompanion_force_marker_.type = visualization_msgs::Marker::ARROW;
  personCompanion_force_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_force_marker_.lifetime = ros::Duration(1.3f);
  personCompanion_force_marker_.scale.x = 0.2;
  personCompanion_force_marker_.scale.y = 0.25;
  personCompanion_force_marker_.color.a = 0.8;
  personCompanion_force_marker_.color.r = 1.0;
  personCompanion_force_marker_.color.g = 0.0;
  personCompanion_force_marker_.color.b = 0.0;
  personCompanion_force_marker_.points.push_back(  geometry_msgs::Point() );
  personCompanion_force_marker_.points.push_back(  geometry_msgs::Point() );
 //ROS_INFO( " 14 init_person_companion_markers() ");
  // marker to goal (blue)
  personCompanion_force_goal_marker_ = personCompanion_force_marker_;
  personCompanion_force_goal_marker_.color.r = 0.0;
  personCompanion_force_goal_marker_.color.g = 0.4;
  personCompanion_force_goal_marker_.color.b = 1.0;

  // marker interaction with persons (green)
  personCompanion_force_int_person_marker_= personCompanion_force_marker_;
  personCompanion_force_int_person_marker_.color.r = 0.2;
  personCompanion_force_int_person_marker_.color.g = 0.85;
  personCompanion_force_int_person_marker_.color.b = 0.2;
 //ROS_INFO( " 15 init_person_companion_markers() ");
  //marker due to obstacles in the scene (black)
  personCompanion_force_obstacle_marker_ = personCompanion_force_marker_;
  personCompanion_force_obstacle_marker_.color.r = 0.0;
  personCompanion_force_obstacle_marker_.color.g = 0.0;
  personCompanion_force_obstacle_marker_.color.b = 0.0;

  // force of interaction with robot (purple)
  personCompanion_force_int_robot_marker_ = personCompanion_force_marker_;
  personCompanion_force_int_robot_marker_.color.r = 0.26;
  personCompanion_force_int_robot_marker_.color.g = 0.0;
  personCompanion_force_int_robot_marker_.color.b = 0.66;

  // marker force companion (cian)
  //force_companion_marker_ = personCompanion_force_marker_;
  //force_companion_marker_.color.r = 0.51;
  //force_companion_marker_.color.g = 0.243;
  //force_companion_marker_.color.b = 0.255;

  // ROS_INFO( " 16 init_person_companion_markers() ");
  //text marker
  personCompanion_text_marker_.header.frame_id = this->fixed_frame;
  personCompanion_text_marker_.ns = "text";
  personCompanion_text_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  personCompanion_text_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_text_marker_.lifetime = ros::Duration(1.3f);
  personCompanion_text_marker_.scale.z = 0.3;
  personCompanion_text_marker_.pose.position.z = 0.0;
  personCompanion_text_marker_.color.a = 1.0;
 //ROS_INFO( " 17 init_person_companion_markers() ");

}

void PeopleSimulationAlgNode::fill_my_covariance_marker( visualization_msgs::Marker& marker, const SpointV_cov& point )
{
  marker.pose.position.x = point.x;
  marker.pose.position.y = point.y;
  marker.pose.position.z = 0.0;

  //plot covariances ellipses
  Eigen::Matrix2d cov;
  cov(0, 0) = point.cov.at(0);//xx
  cov(0, 1) = point.cov.at(1);
  cov(1, 0) = point.cov.at(4);
  cov(1, 1) = point.cov.at(5);//yy

  //if((cov(0, 0)>1)||(cov(0, 1)>1)||(cov(1, 0)>1)||(cov(1, 1)>1)){
   // ROS_INFO(" cov(0, 0) = %f, cov(0, 1) = %f,cov(1, 0) = %f, cov(1, 1) = %f", cov(0, 0),cov(0, 1), cov(1, 0), cov(1, 1));
  // }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);
  //eig.compute(cov);
  marker.scale.x = sqrt( eig.eigenvalues()[0] ) + 0.001;//major vap
  marker.scale.y = sqrt( eig.eigenvalues()[1] ) + 0.001;//minor vap

  //if((marker.scale.x>1)||(marker.scale.y>1)){
   // ROS_INFO(" marker.scale.x = sqrt( eig.eigenvalues()[0] ) + 0.001= %f", marker.scale.x);
   // ROS_INFO(" marker.scale.y = sqrt( eig.eigenvalues()[1] ) + 0.001= %f", marker.scale.y);
  //}
  double angle = atan2( eig.eigenvectors()(1,0), eig.eigenvectors()(0,0) );
  marker.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
  
}

void PeopleSimulationAlgNode::fill_forces_markers() // ok, creo...
{
  const std::list<Cperson_abstract *>* person_list = scene_.get_akp_for_person_companion()->get_scene( );
  Sforce force_to_goal, force_int_person , force_int_robot, force_obstacle, force_total, force_companion;
  geometry_msgs::Point ros_point, ros_point_ini;
  unsigned int cont_f = 0;

  for( std::list<Cperson_abstract*>::const_iterator iit = person_list->begin();
          iit!=person_list->end(); iit++ )
  {
    // fill FORCES for each person
    //Sforce get_forces_person( Sforce&  force_to_goal, Sforce& force_int_person , Sforce& force_int_robot, Sforce& force_obstacle );
    force_total = (*iit)->get_forces_person( force_to_goal, force_int_person , force_int_robot, force_obstacle );
    ros_point_ini.x = (*iit)->get_current_pointV().x;
    ros_point_ini.y = (*iit)->get_current_pointV().y;

    //scaled person interaction force (green)
    personCompanion_force_int_person_marker_.points[0]= ros_point_ini;
    ros_point.x = ros_point_ini.x + force_int_person.fx;
    ros_point.y = ros_point_ini.y + force_int_person.fy;
    personCompanion_force_int_person_marker_.points[1] = ros_point;
    personCompanion_force_int_person_marker_.id = cont_f;
    ++cont_f;
    personCompanion_MarkerArray_msg_.markers.push_back(  personCompanion_force_int_person_marker_  );

    // robot to person interaction force (pink)
    personCompanion_force_int_robot_marker_.points[0]= ros_point_ini;
    ros_point.x = ros_point_ini.x + force_int_robot.fx;
    ros_point.y = ros_point_ini.y + force_int_robot.fy;
    personCompanion_force_int_robot_marker_.points[1] = ros_point;
    personCompanion_force_int_robot_marker_.id = cont_f;
    ++cont_f;
    personCompanion_MarkerArray_msg_.markers.push_back(  personCompanion_force_int_robot_marker_  );

    //map obstacles interaction force (black)
    personCompanion_force_obstacle_marker_.points[0] = ros_point_ini;
    ros_point.x = ros_point_ini.x + force_obstacle.fx;
    ros_point.y = ros_point_ini.y + force_obstacle.fy;
    personCompanion_force_obstacle_marker_.points[1] =  ros_point;
    personCompanion_force_obstacle_marker_.id = cont_f;
    ++cont_f;
    personCompanion_MarkerArray_msg_.markers.push_back(  personCompanion_force_obstacle_marker_  );

  }
  //print robot forces
 
  force_total = scene_.get_akp_for_person_companion()->get_person_companion_akp()->get_forces_person_companion(force_to_goal, force_int_person , force_int_robot, force_obstacle, force_companion);

  ros_point_ini.x = scene_.get_akp_for_person_companion()->get_person_companion_akp()->get_current_pointV().x;
  ros_point_ini.y = scene_.get_akp_for_person_companion()->get_person_companion_akp()->get_current_pointV().y;

  if(debug_antes_subgoals_entre_AKP_goals_){
    ROS_INFO("PeopleSimulationAlgNode::markers, ros_point_ini.x=%f=",ros_point_ini.x);
    ROS_INFO("PeopleSimulationAlgNode::markers, ros_point_ini.y=%f=",ros_point_ini.y);

    ROS_INFO("(ROS) PeopleSimulationAlgNode::markers, force_int_person.fx=%f",force_int_person.fx);
    ROS_INFO("(ROS) PeopleSimulationAlgNode::markers, force_int_person.fy=%f",force_int_person.fy);
  }
  //scaled force to goal: ( blue )
  if(this->person_companion_goal_force_marker)
  {
    if(debug_antes_subgoals_entre_AKP_goals_){
      ROS_INFO("(ROS) robot force goal");
      ROS_INFO("(ROS) ros_point_ini.x=%f",ros_point_ini.x);
      ROS_INFO("(ROS) ros_point_ini.y=%f",ros_point_ini.y);
      ROS_INFO("(ROS) force_to_goal.fx=%f",force_to_goal.fx);
      ROS_INFO("(ROS) force_to_goal.fy=%f",force_to_goal.fy);  
    }

    personCompanion_force_goal_marker_.points[0]= ros_point_ini;
    ros_point.x = ros_point_ini.x + 2*force_to_goal.fx;
    ros_point.y = ros_point_ini.y + 2*force_to_goal.fy;

    if(debug_real_test_companion_){
      ROS_INFO("personCompanion_force_goal_marker_ (ROS) ros_point.x=%f",ros_point.x);
      ROS_INFO("personCompanion_force_goal_marker_ (ROS) ros_point.y=%f",ros_point.y);
    }

    personCompanion_force_goal_marker_.points[1] = ros_point;
    personCompanion_force_goal_marker_.id = cont_f;
    ++cont_f;
    personCompanion_MarkerArray_msg_.markers.push_back(  personCompanion_force_goal_marker_  );
  }

  //scaled person interaction force (green)
  if(this->person_companion_person_forces_marker)
  { 
    if(debug_antes_subgoals_entre_AKP_goals_){
      ROS_INFO("(ROS) robot force person");
      ROS_INFO("(ROS) force_int_person.fx=%f",force_int_person.fx);
      ROS_INFO("(ROS) force_int_person.fy=%f",force_int_person.fy);
    }

    personCompanion_force_int_person_marker_.points[0]= ros_point_ini;
    ros_point.x = ros_point_ini.x + 2*force_int_person.fx;
    ros_point.y = ros_point_ini.y + 2*force_int_person.fy;
    personCompanion_force_int_person_marker_.points[1] = ros_point;
    personCompanion_force_int_person_marker_.id = cont_f;
    ++cont_f;
    personCompanion_MarkerArray_msg_.markers.push_back(  personCompanion_force_int_person_marker_  );
  }

  //map obstacles interaction force (black)
  if(this->person_companion_obstacles_forces_marker)
  {
    if(debug_antes_subgoals_entre_AKP_goals_){
      ROS_INFO("(ROS) robot force obstacles");
    }

    personCompanion_force_obstacle_marker_.points[0] = ros_point_ini;
    ros_point.x = ros_point_ini.x + 2*force_obstacle.fx;
    ros_point.y = ros_point_ini.y + 2*force_obstacle.fy;
    personCompanion_force_obstacle_marker_.points[1] =  ros_point;
    personCompanion_force_obstacle_marker_.id = cont_f;
    ++cont_f;
    personCompanion_MarkerArray_msg_.markers.push_back(  personCompanion_force_obstacle_marker_  );
  }

  //weighted resultant force (red)
  if(this->person_companion_resultant_force_marker)
  {
    if(debug_antes_subgoals_entre_AKP_goals_){
      ROS_INFO("(ROS) robot force resultan");
    }

    personCompanion_force_marker_.points[0] = ros_point_ini;
    ros_point.x = ros_point_ini.x + 2*force_total.fx;
    ros_point.y = ros_point_ini.y + 2*force_total.fy;
    personCompanion_force_marker_.points[1] =  ros_point;
    personCompanion_force_marker_.id = cont_f;
    ++cont_f;
    personCompanion_MarkerArray_msg_.markers.push_back(  personCompanion_force_marker_  );
  }

  //scaled force to goal: ( Cian )
  //if(this->person_companion_companion_force_marker)
  // {
 /*   if(debug_antes_subgoals_entre_AKP_goals_){
      ROS_INFO("(ROS) robot force goal companion");
      ROS_INFO("(ROS) ini ros_point_ini.x=%f",ros_point_ini.x);
      ROS_INFO("(ROS) ini ros_point_ini.y=%f",ros_point_ini.y);
      ROS_INFO("(ROS) force_companion.fx=%f",force_companion.fx);
      ROS_INFO("(ROS) force_companion.fy=%f",force_companion.fy);
    }

    force_companion_marker_.points[0]= ros_point_ini;
    ros_point.x = ros_point_ini.x + 2*force_companion.fx;
    ros_point.y = ros_point_ini.y + 2*force_companion.fy;

   if(debug_real_test_companion_){
      ROS_INFO("force_companion_marker_ (ROS) final ros_point.x=%f",ros_point.x);
      ROS_INFO("force_companion_marker_ (ROS) final ros_point.y=%f",ros_point.y);
    }

    force_companion_marker_.points[1] = ros_point;
    force_companion_marker_.id = cont_f;
    ++cont_f;
    MarkerArray_msg_.markers.push_back(  force_companion_marker_  );
  //}

  // const Crobot* act_robot=scene_.get_akp_for_person_companion()->get_robot();
  Sdestination robot_companion_dest= scene_.get_akp_for_person_companion()->get_person_companion_akp()->get_companion_force_goal();
  //scene_.get_akp_for_person_companion()->get_robot()->get_companion_force_goal(robot_companion_dest);
  force_companion_goal_marker_.pose.position.x = robot_companion_dest.x;
  force_companion_goal_marker_.pose.position.y = robot_companion_dest.y;
  force_companion_goal_marker_.id = 20;
  // MarkerArray_msg_.markers.push_back( force_companion_goal_marker_ );*/
  
}

void PeopleSimulationAlgNode::fill_scene_markers() //ok, creo...
{

  unsigned int cont(0);
  const std::vector<Sdestination>* dest = scene_.get_akp_for_person_companion()->get_destinations();
  for( unsigned int i = 0; i < dest->size(); ++i)
  {
    personCompanion_cylinder_marker_.pose.position.x = dest->at(i).x;
    personCompanion_cylinder_marker_.pose.position.y = dest->at(i).y;
    personCompanion_cylinder_marker_.id = cont;
    personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_cylinder_marker_ );
    ++cont;
  } 

  //draw robot destination, both local and global
  personCompanion_robot_goal_marker_.pose.position.x = scene_.get_akp_for_person_companion()->get_robot_local_goal().x;
  personCompanion_robot_goal_marker_.pose.position.y = scene_.get_akp_for_person_companion()->get_robot_local_goal().y;
  personCompanion_robot_goal_marker_.id = cont;
  ++cont;
  personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_robot_goal_marker_ );
  personCompanion_robot_goal_marker_.pose.position.x = scene_.get_akp_for_person_companion()->get_robot_goal().x;
  personCompanion_robot_goal_marker_.pose.position.y = scene_.get_akp_for_person_companion()->get_robot_goal().y;
  personCompanion_robot_goal_marker_.id = cont;
  ++cont;
  personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_robot_goal_marker_ );
  
  //if slicing mode, plot the set of destinations
  if( person_companion_goal_providing_mode_ == PeopleSimulationAlgNode::Slicing_global )
  {
    for( unsigned int i = 0; i< sliced_global_plan_.size(); ++i )
    {
      personCompanion_robot_subgoal_marker_.pose.position.x = sliced_global_plan_[i].pose.position.x;
      personCompanion_robot_subgoal_marker_.pose.position.y = sliced_global_plan_[i].pose.position.y;
      personCompanion_robot_subgoal_marker_.id = cont;
      ++cont;
      personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_robot_subgoal_marker_ );
    }
  }

  //draw workspace of the robot
  double r = scene_.get_akp_for_person_companion()->get_workspace_radii();
	geometry_msgs::Point ros_point, center_point;
  personCompanion_workspace_marker_.points.clear();
  personCompanion_workspace_marker_.id = cont;
  ++cont;
  const std::vector<Spose>* plans = scene_.get_akp_for_person_companion()->get_person_companion_akp()->get_robot_planning_trajectory();

  if(!plans->empty()){
    center_point.x = plans->at(0).x;
    center_point.y = plans->at(0).y;
  }else{
    center_point.x = robot_pose_.x;
    center_point.y = robot_pose_.y;
  }


  for( unsigned int i = 0; i < 40; ++i )
  {

  /*
  const std::vector<Spose>* plans = scene_.get_akp_for_person_companion()->get_robot()->get_robot_planning_trajectory();
  const std::vector<Sedge_tree>* edges = scene_.get_akp_for_person_companion()->get_plan_edges();
	geometry_msgs::Point ros_point;
  double ini_time = plans->front().time_stamp;
	
	// plot 3d plan, complete tree
  personCompanion_planning_marker_.points.clear();
  for( unsigned int i = 1; i< plans->size(); ++i )
  {
    //lines segments beetwen 2 points
    ros_point.x = plans->at(i).x;
    ros_point.y = plans->at(i).y;
    ros_point.z = plans->at(i).time_stamp - ini_time;
*/
    ros_point.x = center_point.x + r*cos( 2*i*PI/40 );
    ros_point.y = center_point.y + r*sin( 2*i*PI/40 );
    //ros_point.x = robot_pose_.x + r*cos( 2*i*PI/40 );
    //ros_point.y = robot_pose_.y + r*sin( 2*i*PI/40 );
    personCompanion_workspace_marker_.points.push_back( ros_point ); 
    ros_point.x = center_point.x + r*cos( (2*i+1)*PI/40 );
    ros_point.y = center_point.y + r*sin( (2*i+1)*PI/40 );
    //ros_point.x = robot_pose_.x + r*cos( 2*i*PI/40 );
    //ros_point.y = robot_pose_.y + r*sin( 2*i*PI/40 );
    personCompanion_workspace_marker_.points.push_back( ros_point ); 
  }
  personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_workspace_marker_ );

  // robot_goal_marker!!!
  Spoint robot_goal=scene_.get_akp_for_person_companion()->get_robot_goal();
  personCompanion_robot_goal_marker2_.pose.position.x = robot_goal.x;
  personCompanion_robot_goal_marker2_.pose.position.y = robot_goal.y;
  personCompanion_robot_goal_marker2_.id = cont;
  ++cont;
  personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_robot_goal_marker2_ );

  Spoint robot_goal2=scene_.get_akp_for_person_companion()->get_external_robot_goal();
  personCompanion_robot_goal_marker3_.pose.position.x = robot_goal2.x;
  personCompanion_robot_goal_marker3_.pose.position.y = robot_goal2.y;
  personCompanion_robot_goal_marker3_.id = cont;
  ++cont;
  personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_robot_goal_marker3_ );

}

void PeopleSimulationAlgNode::fill_planning_markers_2d()
{
  const std::vector<Spose>* plans = scene_.get_akp_for_person_companion()->get_person_companion_akp()->get_robot_planning_trajectory();
	geometry_msgs::Point ros_point;
  //random goals printing -------------------------------------------------------------------
  const std::vector<Spoint>* goals =  scene_.get_akp_for_person_companion()->get_random_goals();
  for( unsigned int i = 0; i< goals->size(); ++i )
  {
    personCompanion_planning_goals_marker_.pose.position.x = goals->at(i).x;
    personCompanion_planning_goals_marker_.pose.position.y = goals->at(i).y;
    personCompanion_planning_goals_marker_.id = i;
    personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_planning_goals_marker_ );
  }
  
  // plot non-dominated solutions in 2d ..................-------------------------------------
  const std::vector<unsigned int>* nondominated_path_index = scene_.get_akp_for_person_companion()->get_robot_nondominated_plan_index();
  const std::vector<unsigned int>* nondominated_end_of_path_index = scene_.get_akp_for_person_companion()->get_robot_nondominated_end_of_plan_index();
  personCompanion_nd_path2d_marker_.points.clear();
  //ROS_INFO("size = %d" , nondominated_path_index->size());
  for( unsigned int i = 0; i< nondominated_path_index->size(); i++ )
  {
    ros_point.x = plans->at(nondominated_path_index->at(i)).x;
    ros_point.y = plans->at(nondominated_path_index->at(i)).y;
    ros_point.z = 0.0;
    personCompanion_nd_path2d_marker_.points.push_back( ros_point );
  }
  //paths always even
  personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_nd_path2d_marker_ );
  
  // plot text id of paths, if MultiObjective function mode is activated ---------------------
  personCompanion_text_marker_.id = 0;
  unsigned int i = 0;
  for(  ; i< nondominated_end_of_path_index->size(); i++ )
  {
    personCompanion_text_marker_.id++; 
    personCompanion_text_marker_.pose.position.x = plans->at(nondominated_end_of_path_index->at(i)).x;
    personCompanion_text_marker_.pose.position.y = plans->at(nondominated_end_of_path_index->at(i)).y;
    std::stringstream idText;
    idText << nondominated_end_of_path_index->at(i);
    personCompanion_text_marker_.text = idText.str();
    personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_text_marker_ );
  }
  unsigned int text_markers_size = i;
	personCompanion_text_marker_.action = visualization_msgs::Marker::DELETE;
  for( ; i< text_markers_old_size_ ; i++)
  {
    personCompanion_text_marker_.id++;
    personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_text_marker_ );
  }
	personCompanion_text_marker_.action = visualization_msgs::Marker::ADD;
  text_markers_old_size_ = text_markers_size;

}

void PeopleSimulationAlgNode::fill_planning_markers_3d() //ok, creo...
{
  const std::vector<Spose>* plans = scene_.get_akp_for_person_companion()->get_person_companion_akp()->get_robot_planning_trajectory();
  const std::vector<Sedge_tree>* edges = scene_.get_akp_for_person_companion()->get_plan_edges();
  geometry_msgs::Point ros_point;
  double ini_time = plans->front().time_stamp;
	
  // plot 3d plan, complete tree
  personCompanion_planning_marker_.points.clear();
  for( unsigned int i = 1; i< plans->size(); ++i )
  {
    //lines segments beetwen 2 points
    ros_point.x = plans->at(i).x;
    ros_point.y = plans->at(i).y;
    ros_point.z = plans->at(i).time_stamp - ini_time;
    personCompanion_planning_marker_.points.push_back( ros_point );
    //ROS_INFO("message of size %d, at %d and parent %d", plans->size(), i, edges->at(i).parent);
    ros_point.x = plans->at( edges->at(i).parent ).x;
    ros_point.y = plans->at( edges->at(i).parent ).y;
    ros_point.z = plans->at(edges->at(i).parent).time_stamp - ini_time;
    personCompanion_planning_marker_.points.push_back( ros_point );
  }
  personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_planning_marker_ ); 
    
  //plot best trajectory path in 3d
  const std::vector<unsigned int>* best_path_index = scene_.get_akp_for_person_companion()->get_robot_plan_index();
  personCompanion_best_path_marker_.points.clear();
  //ROS_INFO("size = %d" , personCompanion_best_path_marker_.points.size());
  for( unsigned int i = 0; i< best_path_index->size(); i++ )
  {
    ros_point.x = plans->at(best_path_index->at(i)).x;
    ros_point.y = plans->at(best_path_index->at(i)).y;
    ros_point.z = plans->at(best_path_index->at(i)).time_stamp - ini_time;
    personCompanion_best_path_marker_.points.push_back( ros_point );
  }
  personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_best_path_marker_ );
  
  // plot non-dominated solutions in 3d
  const std::vector<unsigned int>* nondominated_path_index = scene_.get_akp_for_person_companion()->get_robot_nondominated_plan_index();
  personCompanion_nd_path_marker_.points.clear();
  //ROS_INFO("size = %d" , nondominated_path_index->size());
  for( unsigned int i = 0; i< nondominated_path_index->size(); i++ )
  {
    ros_point.x = plans->at(nondominated_path_index->at(i)).x;
    ros_point.y = plans->at(nondominated_path_index->at(i)).y;
    ros_point.z = plans->at(nondominated_path_index->at(i)).time_stamp - ini_time;
    personCompanion_nd_path_marker_.points.push_back( ros_point );
  }
  //paths always even
  personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_nd_path_marker_ );

}


void PeopleSimulationAlgNode::fill_people_prediction_markers_2d()  // ok, creo...
{
  //plot people trajectories 2d
  const std::vector<unsigned int>* best_path_index = scene_.get_akp_for_person_companion()->get_robot_plan_index();
  const std::list<Cperson_abstract *>* person_list = scene_.get_akp_for_person_companion()->get_scene( );
  const std::vector<SpointV_cov>* traj;
  unsigned int cont_2d = 0;
  for( std::list<Cperson_abstract*>::const_iterator iit = person_list->begin();
        iit!=person_list->end(); iit++ )
  {
    traj = (*iit)->get_planning_trajectory();
    unsigned int id_person=(*iit)->get_id();
    //ROS_INFO("id_person= %d", id_person);


    //if((id_person==1000)||(id_person==1015)||(id_person==1019)||(id_person==774)){ //||(id_person==733)
      if ( traj->size() > best_path_index->size() )
      //if ( traj->empty() )
      {

        unsigned int less_prediction= (best_path_index->size())/2;

          //ROS_INFO("traj->size()= %d", traj->size());
          //ROS_INFO("best_path_index->size()= %d", best_path_index->size());
          //ROS_INFO("less_prediction= %d", less_prediction);
        for( unsigned int i = 0; i< best_path_index->size(); ++i , ++cont_2d)
        //for( unsigned int i = 0; i< best_path_index->size()-less_prediction; ++i , ++cont_2d)
        {
          fill_my_covariance_marker( personCompanion_pred_traj2d_marker_,  traj->at(best_path_index->at(i)) );
          personCompanion_pred_traj2d_marker_.id = cont_2d;
          personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_pred_traj2d_marker_ );
        }
      }
    //}

  }

}



void PeopleSimulationAlgNode::fill_people_prediction_markers_3d() // ok, creo...
{

  const std::list<Cperson_abstract *>* person_list = scene_.get_akp_for_person_companion()->get_scene( );
  const std::vector<SpointV_cov>* traj;
  unsigned int cont = 0;
  double time_stamp_ini = scene_.get_akp_for_person_companion()->get_time();


  //ROS_INFO( "person list %d" , person_list->size() );
  for( std::list<Cperson_abstract*>::const_iterator iit = person_list->begin();
          iit!=person_list->end(); iit++ )
  {
    if( (*iit)->get_planning_trajectory()->size() <= 1 )
      traj = (*iit)->get_prediction_trajectory();
    else
      traj = (*iit)->get_planning_trajectory();

    unsigned int less_prediction= 13;//= (traj->size())/2;

   // ROS_INFO("traj->size()= %d", traj->size());
    //ROS_INFO("less_prediction= %d", less_prediction);

    unsigned int id_person=(*iit)->get_id();
    //ROS_INFO("id_person= %d", id_person);
    //if((id_person==476)||(id_person==773)||(id_person==775)||(id_person==774)){ //||(id_person==733)
      //fill prediction covariances
      for( unsigned int i = 0; i< traj->size(); ++i , ++cont)
     // for( unsigned int i = 0; i< less_prediction; ++i , ++cont)
      {
         //ROS_INFO("i= %d", i);
        fill_my_covariance_marker( personCompanion_pred_traj_marker_,  traj->at(i) );
        personCompanion_pred_traj_marker_.pose.position.z = traj->at(i).time_stamp - time_stamp_ini;//2d cov elevated the time azis(z)
        personCompanion_pred_traj_marker_.id = cont;
        personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_pred_traj_marker_ );
      }
    //}

  }

}


void PeopleSimulationAlgNode::fill_laser_obstacles() // ok, creo...
{
  unsigned int cont(0);
  const std::vector<Spoint>* obstacles = scene_.get_akp_for_person_companion()->get_laser_obstacles();
  for( unsigned int i = 0; i < obstacles->size(); ++i)
  {
    personCompanion_laser_obstacle_marker_.pose.position.x = obstacles->at(i).x;
    personCompanion_laser_obstacle_marker_.pose.position.y = obstacles->at(i).y;
    personCompanion_laser_obstacle_marker_.id = cont;
    ++cont;
    personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_laser_obstacle_marker_ );
  }

}

void PeopleSimulationAlgNode::fill_best_path_2d()  // ok
{
  geometry_msgs::Point ros_point;
  personCompanion_best_path2d_marker_.points.clear();
  const std::vector<Spose>* plans = scene_.get_akp_for_person_companion()->get_person_companion_akp()->get_robot_planning_trajectory();
  const std::vector<unsigned int>* best_path_index = scene_.get_akp_for_person_companion()->get_robot_plan_index(); //OK
  //const std::vector<Spose>* best_plan = scene_.get_akp_for_person_companion()->get_best_planning_trajectory();

  //ROS_ERROR("best path size = %d, and robot plan size = %d" , best_path_index->size(), plans->size()  );
  for( unsigned int i = 0; i< best_path_index->size(); i++ )
  {
    //ROS_INFO ( "size best_path = %d, and size of plans %d ", best_path_index->size(), plans->size()  );
    //ROS_INFO("filling %d, and best path index = %d" , i, best_path_index->at(i));
    ros_point.x = plans->at(best_path_index->at(i)).x;
    ros_point.y = plans->at(best_path_index->at(i)).y;
    ros_point.z = 0.0;
    personCompanion_best_path2d_marker_.points.push_back( ros_point );
  }

  personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_best_path2d_marker_ );
}


