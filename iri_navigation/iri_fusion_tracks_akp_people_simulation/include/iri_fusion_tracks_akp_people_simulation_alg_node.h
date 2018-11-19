// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _iri_fusion_tracks_akp_people_simulation_alg_node_h_
#define _iri_fusion_tracks_akp_people_simulation_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "iri_fusion_tracks_akp_people_simulation_alg.h"

// [publisher subscriber headers]
#include <visualization_msgs/MarkerArray.h>
#include <iri_perception_msgs/detectionArray.h>

// [service client headers]

// [action server client headers]


// my libs/includes
// [libraries]
#include "nav/plan_local_nav.h"

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h> // robot companion (ely) trasladar modelo tibi robot companion al marcker donde deberia estar el robot.

#include <random>
#include <iostream>
#include <fstream>      // std::ofstream
#include <string>
#include <stdio.h>
using namespace std;



/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class IriFusionTracksAkpPeopleSimulationAlgNode : public algorithm_base::IriBaseAlgorithm<IriFusionTracksAkpPeopleSimulationAlgorithm>
{
  private:
    // [publisher attributes]
    ros::Publisher MarkerArray_out_people_tracks_publisher_;
    visualization_msgs::MarkerArray MarkerArray_out_people_tracks_MarkerArray_msg_;

    ros::Publisher fusion_simulated_people_tracks_publisher_;
    iri_perception_msgs::detectionArray fusion_simulated_people_tracks_detectionArray_msg_;


    // [subscriber attributes]
    ros::Subscriber dabo2_tracks_subscriber_;
    void dabo2_tracks_callback(const iri_perception_msgs::detectionArray::ConstPtr& msg);
    pthread_mutex_t dabo2_tracks_mutex_;
    void dabo2_tracks_mutex_enter(void);
    void dabo2_tracks_mutex_exit(void);

    ros::Subscriber dabo1_tracks_subscriber_;
    void dabo1_tracks_callback(const iri_perception_msgs::detectionArray::ConstPtr& msg);
    pthread_mutex_t dabo1_tracks_mutex_;
    void dabo1_tracks_mutex_enter(void);
    void dabo1_tracks_mutex_exit(void);


    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]

    // my variables:

    //ros::Publisher tracks_publisher_;
    iri_perception_msgs::detectionArray peopleTrackingArray_msg_;

    visualization_msgs::MarkerArray personCompanion_MarkerArray_msg_;
    visualization_msgs::Marker personCompanion_text_marker_,
      person_track_marker_;

    std::vector<SdetectionObservation> obs_dabo_;
    std::vector<SdetectionObservation> obs_dabo2_;
    std::vector<SdetectionObservation> tracks_fused_;

    bool we_habe_tracks_of_dabo_;
    bool we_habe_tracks_of_dabo2_;

    int id_dabo_;
    int id_dabo2_;
    std::string fixed_frame;

   void init_fusion_tracks_simul_pers_markers();

   /**
    * \brief config variable
    *
    * This variable has all the driver parameters defined in the cfg config file.
    * Is updated everytime function config_update() is called.
    */
    Config config_;
  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    IriFusionTracksAkpPeopleSimulationAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~IriFusionTracksAkpPeopleSimulationAlgNode(void);

  protected:
   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the algorithm is on running state. Loop frequency 
    * can be tuned by modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects 
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
    void mainNodeThread(void);

   /**
    * \brief dynamic reconfigure server callback
    * 
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must 
    * implement it.
    *
    * \param config an object with new configuration from all algorithm 
    *               parameters defined in the config file.
    * \param level  integer referring the level in which the configuration
    *               has been changed.
    */
    void node_config_update(Config &config, uint32_t level);

   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the 
    * specific algorithms may be added.
    */
    void addNodeDiagnostics(void);

    // [diagnostic functions]
    
    // [test functions]
};

#endif
