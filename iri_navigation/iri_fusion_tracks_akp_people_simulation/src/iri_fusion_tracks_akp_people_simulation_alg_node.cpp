#include "iri_fusion_tracks_akp_people_simulation_alg_node.h"
#include <pluginlib/class_list_macros.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

IriFusionTracksAkpPeopleSimulationAlgNode::IriFusionTracksAkpPeopleSimulationAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<IriFusionTracksAkpPeopleSimulationAlgorithm>(),
  we_habe_tracks_of_dabo_(false),
  we_habe_tracks_of_dabo2_(false)
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  this->MarkerArray_out_people_tracks_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("MarkerArray_out_people_tracks", 100);
  this->fusion_simulated_people_tracks_publisher_ = this->public_node_handle_.advertise<iri_perception_msgs::detectionArray>("fusion_simulated_people_tracks", 100);
  
  // [init subscribers]
  this->dabo2_tracks_subscriber_ = this->public_node_handle_.subscribe("dabo2_tracks", 100, &IriFusionTracksAkpPeopleSimulationAlgNode::dabo2_tracks_callback, this);
  pthread_mutex_init(&this->dabo2_tracks_mutex_,NULL);

  this->dabo1_tracks_subscriber_ = this->public_node_handle_.subscribe("dabo1_tracks", 100, &IriFusionTracksAkpPeopleSimulationAlgNode::dabo1_tracks_callback, this);
  pthread_mutex_init(&this->dabo1_tracks_mutex_,NULL);

   init_fusion_tracks_simul_pers_markers(); 
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

IriFusionTracksAkpPeopleSimulationAlgNode::~IriFusionTracksAkpPeopleSimulationAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->dabo2_tracks_mutex_);
  pthread_mutex_destroy(&this->dabo1_tracks_mutex_);
}

void IriFusionTracksAkpPeopleSimulationAlgNode::mainNodeThread(void)
{
  ROS_INFO(" IN!!!! IriFusionTracksAkpPeopleSimulationAlgNode");
  this->fixed_frame = "/map";
  // [fill msg structures]
  // Initialize the topic message structure
  //this->MarkerArray_vision_out_people_tracks_MarkerArray_msg_.data = my_var;

  // Initialize the topic message structure
  //this->fusion_simulated_people_tracks_detectionArray_msg_.data = my_var;
  this->dabo1_tracks_mutex_enter();
  this->dabo2_tracks_mutex_enter();
  tracks_fused_.clear();
  if((we_habe_tracks_of_dabo_)&&(we_habe_tracks_of_dabo2_)){
	  ROS_INFO(" WE HAVE TRACKS OF BOTH ROBOTS");
    for( unsigned int i = 0; i< obs_dabo_.size(); ++i)
    {
      ROS_INFO(" obs_dabo_[i].id=%d",obs_dabo_[i].id);
	    if(obs_dabo_[i].id!=id_dabo2_){
	      tracks_fused_.push_back(obs_dabo_[i]);
    	}
    }
    for( unsigned int i = 0; i< obs_dabo2_.size(); ++i)
    {
      ROS_INFO(" obs_dabo2_[i].id=%d",obs_dabo2_[i].id);
	    if(obs_dabo2_[i].id!=id_dabo_){
        if(obs_dabo2_[i].id==id_dabo2_){
          tracks_fused_.push_back(obs_dabo2_[i]);
        }else{
          int adition=10;        
          obs_dabo2_[i].id=obs_dabo2_[i].id+adition;
	        tracks_fused_.push_back(obs_dabo2_[i]);
        }
    	}
    }
	
    we_habe_tracks_of_dabo_=false;
    we_habe_tracks_of_dabo2_=false;
  }
  this->dabo1_tracks_mutex_exit();
  this->dabo2_tracks_mutex_exit();

  this->personCompanion_MarkerArray_msg_.markers.clear();
  peopleTrackingArray_msg_.header.stamp = ros::Time::now();
  peopleTrackingArray_msg_.detection.clear();
  iri_perception_msgs::detection person;

// person_track_marker_.header.frame_id = this->fixed_frame;
//peopleTrackingArray_msg_
peopleTrackingArray_msg_.header.frame_id = "/map";

  for( unsigned int i = 0; i< tracks_fused_.size(); ++i)
  {
      
       person.id = tracks_fused_[i].id;
       person.position.x = tracks_fused_[i].x;
       person.position.y = tracks_fused_[i].y;
       person.velocity.x = tracks_fused_[i].vx;
       person.velocity.y = tracks_fused_[i].vy;
       //ROS_INFO("EEEEEEEEEEE (nodo PERSON sim) VELOCITY published vx=%f ; vy=%f. ",person.velocity.x,person.velocity.y);
       // std::cout << "   EEEEEEEEe    (nodo PERSON sim) VELOCITY published vx="<<person.velocity.x<<"; vy="<<person.velocity.y<< std::endl;
       //covariances conversion from 4x4 to 6x6
       person.covariances[0] = tracks_fused_[i].cov[0];//x x
       person.covariances[1] = tracks_fused_[i].cov[1];//x y
       person.covariances[3] = tracks_fused_[i].cov[2];//x vx
       person.covariances[4] = tracks_fused_[i].cov[3];//x vy
       person.covariances[6] = tracks_fused_[i].cov[4];//x y
       person.covariances[7] = tracks_fused_[i].cov[5];//y y
       person.covariances[9] = tracks_fused_[i].cov[6];//y vx
       person.covariances[10] = tracks_fused_[i].cov[7];//y vy
       person.covariances[18] = tracks_fused_[i].cov[8];//vx x
       person.covariances[19] = tracks_fused_[i].cov[9];//vx y
       person.covariances[21] = tracks_fused_[i].cov[10];//vx vx
       person.covariances[22] = tracks_fused_[i].cov[11];//vx vy
       person.covariances[24] = tracks_fused_[i].cov[12];//vy x
       person.covariances[25] = tracks_fused_[i].cov[13];//vy y
       person.covariances[27] = tracks_fused_[i].cov[14];//vy vx
       person.covariances[28] = tracks_fused_[i].cov[15];//vy vy
       peopleTrackingArray_msg_.detection.push_back(person);


       person_track_marker_.header.frame_id = this->fixed_frame;
       personCompanion_text_marker_.header.frame_id = this->fixed_frame;


       person_track_marker_.id = tracks_fused_[i].id;
       person_track_marker_.pose.position.x = tracks_fused_[i].x;
       person_track_marker_.pose.position.y = tracks_fused_[i].y;
       person_track_marker_.pose.position.z = 0.0;
       person_track_marker_.pose.position.z = 0.3;
       if((tracks_fused_[i].id==id_dabo_)||(tracks_fused_[i].id==id_dabo2_)){
         person_track_marker_.color.r = 0.0;
         person_track_marker_.color.g = 0.5;
         person_track_marker_.color.b = 0.6;
       }else{
         person_track_marker_.color.r = 0.3;
         person_track_marker_.color.g = 1.0;
         person_track_marker_.color.b = 0.3;
       }
       //plot covariances ellipses
       Eigen::Matrix2d cov;
       cov(0, 0) = tracks_fused_[i].cov[0];//xx
       cov(0, 1) = tracks_fused_[i].cov[1];
       cov(1, 0) = tracks_fused_[i].cov[4];
       cov(1, 1) = tracks_fused_[i].cov[5];//yy
       Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);
       eig.compute(cov);
       ROS_INFO(" WE HAVE TRACKS OF BOTH ROBOTS; tracks_fused_[i].cov[0]=%f ; cov[1]= %f ; cov[4]=%f ; cov[5]= %f", tracks_fused_[i].cov[0], tracks_fused_[i].cov[1],tracks_fused_[i].cov[4],tracks_fused_[i].cov[5]);
       person_track_marker_.scale.x = sqrt( eig.eigenvalues()[0] ) + 0.001;//major vap variable size with covariance
       person_track_marker_.scale.y = sqrt( eig.eigenvalues()[1] ) + 0.001;//minor vap
       person_track_marker_.scale.z = 0.3;

       person_track_marker_.scale.x = 0.5; // fixed size
       person_track_marker_.scale.y = 0.5;
       person_track_marker_.scale.z = 0.6;
       double angle = atan2( eig.eigenvectors()(1,0), eig.eigenvectors()(0,0) );
       person_track_marker_.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
       this->personCompanion_MarkerArray_msg_.markers.push_back( person_track_marker_ ); 


	
      personCompanion_text_marker_.id=tracks_fused_[i].id; 
      personCompanion_text_marker_.pose.position.x = tracks_fused_[i].x;
      personCompanion_text_marker_.pose.position.y = tracks_fused_[i].y;
      std::stringstream idText;
      idText << tracks_fused_[i].id;
      personCompanion_text_marker_.text = idText.str();    
      this->personCompanion_MarkerArray_msg_.markers.push_back( personCompanion_text_marker_ );
      
  }

  if(!tracks_fused_.empty()){
    this->fusion_simulated_people_tracks_publisher_.publish(this->peopleTrackingArray_msg_);
    this->MarkerArray_out_people_tracks_publisher_.publish(this->personCompanion_MarkerArray_msg_);
  }
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  // Uncomment the following line to publish the topic message
  //this->MarkerArray_vision_out_people_tracks_publisher_.publish(this->MarkerArray_vision_out_people_tracks_MarkerArray_msg_);

  // Uncomment the following line to publish the topic message
  //this->fusion_simulated_people_tracks_publisher_.publish(this->fusion_simulated_people_tracks_detectionArray_msg_);

}

void IriFusionTracksAkpPeopleSimulationAlgNode::init_fusion_tracks_simul_pers_markers(){

  person_track_marker_.header.frame_id = this->fixed_frame;
  person_track_marker_.ns = "trackMarcker";
  person_track_marker_.type = visualization_msgs::Marker::CYLINDER;
  person_track_marker_.action = visualization_msgs::Marker::ADD;
  person_track_marker_.lifetime = ros::Duration(1.23f);
  person_track_marker_.scale.z = 0.05;
  person_track_marker_.color.a = 0.6;
  person_track_marker_.color.r = 0.3;
  person_track_marker_.color.g = 1.0;
  person_track_marker_.color.b = 0.3;
  person_track_marker_.pose.orientation.w = 1.0;
  person_track_marker_.pose.position.z = 0.025;

	/*track_marker_.header.frame_id = "/map";
	track_marker_.ns = "tracks";
	track_marker_.type = visualization_msgs::Marker::CYLINDER;
	track_marker_.action = visualization_msgs::Marker::ADD;
	track_marker_.lifetime = ros::Duration(1.0f);*/

  personCompanion_text_marker_.header.frame_id = this->fixed_frame;
  personCompanion_text_marker_.ns = "textID";
  personCompanion_text_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  personCompanion_text_marker_.action = visualization_msgs::Marker::ADD;
  personCompanion_text_marker_.lifetime = ros::Duration(1.3f);
  personCompanion_text_marker_.scale.z = 0.5;
  personCompanion_text_marker_.pose.position.z = 1.0;
  personCompanion_text_marker_.color.a = 0.5;
  personCompanion_text_marker_.color.r = 0.0;
  personCompanion_text_marker_.color.g = 0.0;
  personCompanion_text_marker_.color.b = 0.0;

}
/*  [subscriber callbacks] */
void IriFusionTracksAkpPeopleSimulationAlgNode::dabo2_tracks_callback(const iri_perception_msgs::detectionArray::ConstPtr& msg)
{
  ROS_INFO("IriFusionTracksAkpPeopleSimulationAlgNode::dabo2_tracks_callback: New Message Received");
  ROS_INFO("IriFusionTracksAkpPeopleSimulationAlgNode(dabo2)::tracks_callback: tracks are in %s frame instead of %s", msg->header.frame_id.c_str(), this->fixed_frame.c_str());
  this->fixed_frame = "/map";
  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->dabo2_tracks_mutex_enter();

// ROS_INFO("IriFusionTracksAkpPeopleSimulationAlgNode::tracks_callback: New Message Received"); 
  
  //ROS_INFO("IriFusionTracksAkpPeopleSimulationAlgNode::tracks_callback: msg->header.frame_id= %s",msg->header.frame_id.c_str());
  //ROS_INFO("IriFusionTracksAkpPeopleSimulationAlgNode::tracks_callback: this->fixed_frame= %s",this->fixed_frame.c_str());

  if(msg->header.frame_id == this->fixed_frame)
  {
    if(!this->we_habe_tracks_of_dabo2_){
	    this->we_habe_tracks_of_dabo2_=true;
	    ROS_INFO(" WE HAVE TRACKS OF dabo 2");	
	    this->dabo2_tracks_mutex_enter();
	    //std::vector<SdetectionObservation> obs;
	    this->obs_dabo2_.clear();
	    std::vector<double> cov;
	    cov.resize(16,0.0);
	    //ROS_INFO("IriFusionTracksAkpPeopleSimulationAlgNode::tracks_callback: msg->detection.size() = %d",msg->detection.size());
	    for( unsigned int i = 0; i< msg->detection.size(); ++i)
	    {
	     // ROS_INFO(" [IN ROBOT TRACKS!!!] IriFusionTracksAkpPeopleSimulationAlgNode::tracks_callback: id = %d, (x,y) = (%f, %f), (vx,vy) = (%f, %f), prob=%f", msg->detection[i].id, msg->detection[i].position.x, msg->detection[i].position.y, msg->detection[i].velocity.x, msg->detection[i].velocity.y, msg->detection[i].probability);
	      //covariances conversion from 4x4 to 6x6
	      cov[0] = msg->detection[i].covariances[0];//x x
	      cov[1] = msg->detection[i].covariances[1];//x y
	      cov[2] = msg->detection[i].covariances[3];//x vx
	      cov[3] = msg->detection[i].covariances[4];//x vy
	      cov[4] = msg->detection[i].covariances[6];//x y
	      cov[5] = msg->detection[i].covariances[7];//y y
	      cov[6] = msg->detection[i].covariances[9];//y vx
	      cov[7] = msg->detection[i].covariances[10];//y vy
	      cov[8] = msg->detection[i].covariances[18];//vx x
	      cov[9] = msg->detection[i].covariances[19];//vx y
	      cov[10] = msg->detection[i].covariances[21];//vx vx
	      cov[11] = msg->detection[i].covariances[22];//vx vy
	      cov[12] = msg->detection[i].covariances[24];//vy x
	      cov[13] = msg->detection[i].covariances[25];//vy y
	      cov[14] = msg->detection[i].covariances[27];//vy vx
	      cov[15] = msg->detection[i].covariances[28];//vy vy
	      this->obs_dabo2_.push_back(SdetectionObservation(msg->detection[i].id, msg->header.stamp.toSec(),
		                                  msg->detection[i].position.x,  msg->detection[i].position.y ,
		                                  msg->detection[i].velocity.x, msg->detection[i].velocity.y, cov));
	    }
	    //this->planner_.update_scene(this->obs);//filter = false
	    this->dabo2_tracks_mutex_exit();
    }
    //ROS_INFO("AkpLocalPlanner::tracks_callback: Exit"); 
  }
  else
  {
    ROS_ERROR("IriFusionTracksAkpPeopleSimulationAlgNode(dabo2)::tracks_callback: tracks are in %s frame instead of %s", msg->header.frame_id.c_str(), this->fixed_frame.c_str());
  }



  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->dabo2_tracks_mutex_exit();
}

void IriFusionTracksAkpPeopleSimulationAlgNode::dabo2_tracks_mutex_enter(void)
{
  pthread_mutex_lock(&this->dabo2_tracks_mutex_);
}

void IriFusionTracksAkpPeopleSimulationAlgNode::dabo2_tracks_mutex_exit(void)
{
  pthread_mutex_unlock(&this->dabo2_tracks_mutex_);
}

void IriFusionTracksAkpPeopleSimulationAlgNode::dabo1_tracks_callback(const iri_perception_msgs::detectionArray::ConstPtr& msg)
{
  ROS_INFO("IriFusionTracksAkpPeopleSimulationAlgNode::dabo1_tracks_callback: New Message Received");
  ROS_INFO("IriFusionTracksAkpPeopleSimulationAlgNode(dabo1)::tracks_callback: tracks are in %s frame instead of %s", msg->header.frame_id.c_str(), this->fixed_frame.c_str());
  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->dabo1_tracks_mutex_enter();
  this->fixed_frame = "/map";
// ROS_INFO("AkpLocalPlanner::tracks_callback: New Message Received"); 
  
  //ROS_INFO("IriFusionTracksAkpPeopleSimulationAlgNode::tracks_callback: msg->header.frame_id= %s",msg->header.frame_id.c_str());
  //ROS_INFO("IriFusionTracksAkpPeopleSimulationAlgNode::tracks_callback: this->fixed_frame= %s",this->fixed_frame.c_str());

  if(msg->header.frame_id == this->fixed_frame)
  {
    if(!this->we_habe_tracks_of_dabo_){
	    this->we_habe_tracks_of_dabo_=true;
		ROS_INFO(" WE HAVE TRACKS OF dabo");	
	    this->dabo1_tracks_mutex_enter();
	    //std::vector<SdetectionObservation> obs_dabo;
	    this->obs_dabo_.clear();
	    std::vector<double> cov;
	    cov.resize(16,0.0);
	    //ROS_INFO("IriFusionTracksAkpPeopleSimulationAlgNode::tracks_callback: msg->detection.size() = %d",msg->detection.size());
	    for( unsigned int i = 0; i< msg->detection.size(); ++i)
	    {
	     // ROS_INFO(" [IN ROBOT TRACKS!!!] AkpLocalPlanner::tracks_callback: id = %d, (x,y) = (%f, %f), (vx,vy) = (%f, %f), prob=%f", msg->detection[i].id, msg->detection[i].position.x, msg->detection[i].position.y, msg->detection[i].velocity.x, msg->detection[i].velocity.y, msg->detection[i].probability);
	      //covariances conversion from 4x4 to 6x6
	      cov[0] = msg->detection[i].covariances[0];//x x
	      cov[1] = msg->detection[i].covariances[1];//x y
	      cov[2] = msg->detection[i].covariances[3];//x vx
	      cov[3] = msg->detection[i].covariances[4];//x vy
	      cov[4] = msg->detection[i].covariances[6];//x y
	      cov[5] = msg->detection[i].covariances[7];//y y
	      cov[6] = msg->detection[i].covariances[9];//y vx
	      cov[7] = msg->detection[i].covariances[10];//y vy
	      cov[8] = msg->detection[i].covariances[18];//vx x
	      cov[9] = msg->detection[i].covariances[19];//vx y
	      cov[10] = msg->detection[i].covariances[21];//vx vx
	      cov[11] = msg->detection[i].covariances[22];//vx vy
	      cov[12] = msg->detection[i].covariances[24];//vy x
	      cov[13] = msg->detection[i].covariances[25];//vy y
	      cov[14] = msg->detection[i].covariances[27];//vy vx
	      cov[15] = msg->detection[i].covariances[28];//vy vy
	      this->obs_dabo_.push_back(SdetectionObservation(msg->detection[i].id, msg->header.stamp.toSec(),
		                                  msg->detection[i].position.x,  msg->detection[i].position.y ,
		                                  msg->detection[i].velocity.x, msg->detection[i].velocity.y, cov));
	    }
	    //this->planner_.update_scene(this->obs);//filter = false
	    this->dabo1_tracks_mutex_exit();
	    //ROS_INFO("IriFusionTracksAkpPeopleSimulationAlgNode::tracks_callback: Exit"); 
    }
  }
  else
  {
    ROS_ERROR("IriFusionTracksAkpPeopleSimulationAlgNode(dabo1)::tracks_callback: tracks are in %s frame instead of %s", msg->header.frame_id.c_str(), this->fixed_frame.c_str());
  }
  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->dabo1_tracks_mutex_exit();
}

void IriFusionTracksAkpPeopleSimulationAlgNode::dabo1_tracks_mutex_enter(void)
{
  pthread_mutex_lock(&this->dabo1_tracks_mutex_);
}

void IriFusionTracksAkpPeopleSimulationAlgNode::dabo1_tracks_mutex_exit(void)
{
  pthread_mutex_unlock(&this->dabo1_tracks_mutex_);
}


/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void IriFusionTracksAkpPeopleSimulationAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  id_dabo_=config.id_person_companion_dabo;
  id_dabo2_=config.id_person_companion_dabo2;
  
  this->alg_.unlock();
}

void IriFusionTracksAkpPeopleSimulationAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<IriFusionTracksAkpPeopleSimulationAlgNode>(argc, argv, "iri_fusion_tracks_akp_people_simulation_alg_node");
}
