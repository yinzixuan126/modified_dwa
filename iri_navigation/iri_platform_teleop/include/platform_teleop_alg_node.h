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

#ifndef _platform_teleop_alg_node_h_
#define _platform_teleop_alg_node_h_

// #define PS3_BUTTON_SELECT            0
// #define PS3_BUTTON_STICK_LEFT        1
// #define PS3_BUTTON_STICK_RIGHT       2
// #define PS3_BUTTON_START             3
// #define PS3_BUTTON_CROSS_UP          4
// #define PS3_BUTTON_CROSS_RIGHT       5
// #define PS3_BUTTON_CROSS_DOWN        6
// #define PS3_BUTTON_CROSS_LEFT        7
// #define PS3_BUTTON_REAR_LEFT_2       8
// #define PS3_BUTTON_REAR_RIGHT_2      9
// #define PS3_BUTTON_REAR_LEFT_1       10
// #define PS3_BUTTON_REAR_RIGHT_1      11
// #define PS3_BUTTON_ACTION_TRIANGLE   12
// #define PS3_BUTTON_ACTION_CIRCLE     13
// #define PS3_BUTTON_ACTION_CROSS      14
// #define PS3_BUTTON_ACTION_SQUARE     15
// #define PS3_BUTTON_PAIRING           16
// 
// #define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
// #define PS3_AXIS_STICK_LEFT_UPWARDS      1
// #define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
// #define PS3_AXIS_STICK_RIGHT_UPWARDS     3
// #define PS3_AXIS_BUTTON_CROSS_UP         4
// #define PS3_AXIS_BUTTON_CROSS_RIGHT      5
// #define PS3_AXIS_BUTTON_CROSS_DOWN       6
// #define PS3_AXIS_BUTTON_CROSS_LEFT       7
// #define PS3_AXIS_BUTTON_REAR_LEFT_2      8
// #define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
// #define PS3_AXIS_BUTTON_REAR_LEFT_1      10
// #define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
// #define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
// #define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
// #define PS3_AXIS_BUTTON_ACTION_CROSS     14
// #define PS3_AXIS_BUTTON_ACTION_SQUARE    15
// #define PS3_AXIS_ACCELEROMETER_LEFT      16
// #define PS3_AXIS_ACCELEROMETER_FORWARD   17
// #define PS3_AXIS_ACCELEROMETER_UP        18
// #define PS3_AXIS_GYRO_YAW                19
// 
// #define BUTTON_DEAD_MAN              PS3_BUTTON_REAR_RIGHT_1
// #define BUTTON_TRANS_SPEED_UP        PS3_BUTTON_ACTION_TRIANGLE
// #define BUTTON_TRANS_SPEED_DOWN      PS3_BUTTON_ACTION_CROSS
// #define BUTTON_ROT_SPEED_UP          PS3_BUTTON_ACTION_CIRCLE
// #define BUTTON_ROT_SPEED_DOWN        PS3_BUTTON_ACTION_SQUARE
// #define AXIS_TRANS_FORWARD           PS3_AXIS_STICK_LEFT_UPWARDS
// #define AXIS_ROT_LEFTWARD            PS3_AXIS_STICK_RIGHT_LEFTWARDS

#include <iri_common_drivers_msgs/ps3state.h>
#define BUTTON_CANCEL_GOAL           iri_common_drivers_msgs::ps3state::PS3_BUTTON_SELECT
#define BUTTON_DEAD_MAN              iri_common_drivers_msgs::ps3state::PS3_BUTTON_REAR_RIGHT_1     //R1
#define BUTTON_TRANS_SPEED_UP        iri_common_drivers_msgs::ps3state::PS3_BUTTON_ACTION_TRIANGLE
#define BUTTON_TRANS_SPEED_DOWN      iri_common_drivers_msgs::ps3state::PS3_BUTTON_ACTION_CROSS
#define BUTTON_ROT_SPEED_UP          iri_common_drivers_msgs::ps3state::PS3_BUTTON_ACTION_CIRCLE
#define BUTTON_ROT_SPEED_DOWN        iri_common_drivers_msgs::ps3state::PS3_BUTTON_ACTION_SQUARE
#define AXIS_TRANS_FORWARD           iri_common_drivers_msgs::ps3state::PS3_AXIS_STICK_LEFT_UPWARDS    //L3
#define AXIS_ROT_LEFTWARD            iri_common_drivers_msgs::ps3state::PS3_AXIS_STICK_RIGHT_LEFTWARDS //R3

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "platform_teleop_alg.h"

// [publisher subscriber headers]
#include <actionlib_msgs/GoalID.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class PlatformTeleopAlgNode : public algorithm_base::IriBaseAlgorithm<PlatformTeleopAlgorithm>
{
  private:
    // [publisher attributes]
    ros::Publisher cancel_goal_publisher_;
    actionlib_msgs::GoalID cancel_goal_GoalID_msg_;

    ros::Publisher cmd_vel_publisher_;
    geometry_msgs::Twist twist_msg;

    // [subscriber attributes]
    ros::Subscriber joy_subscriber_;
    void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg);
    pthread_mutex_t joy_mutex_;
    void joy_mutex_enter(void);
    void joy_mutex_exit(void);
    
    void useWii(std::vector<int> current_buttons, std::vector<int> last_buttons);
    void useWiiButton(const unsigned int & index);
    void usePs3(std::vector<int> current_buttons, std::vector<int> last_buttons, std::vector<float> current_axes);
    void usePs3Button(const unsigned int & index);
    
    double vt_max;
    double vr_max;
    double trans_speed_scale;
    double rot_speed_scale;
    bool cancel_goal;

    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]

    /**
    * \brief Tells if the human user is alive
    *
    * Depending on the button B this will be true or false. This is an option 
    * through check_human_
    */
    bool human_is_alive_;
    
   /**
    * \brief config variable
    *
    * This variable has all the parameters defined in the cfg config file.
    * Is updated everytime function node_config_update() is called.
    */
    Config config;
    
    /**
    * \brief Resets joy_watchdog time
    */
    void reset_joy_watchdog(void);
    /**
    * \brief Returns true if joy_watchdog timeouts
    */
    bool joy_watchdog_active(void);
    /**
    * \brief Updates joy_watchdog time
    */
    void update_joy_watchdog(void);
    /**
    * \brief Watchdog timeout duration
    */
    ros::Duration joy_watchdog_duration;
    /**
    * \brief Watchdog access mutex
    */
    CMutex joy_watchdog_access;

  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    PlatformTeleopAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~PlatformTeleopAlgNode(void);

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
};

#endif
