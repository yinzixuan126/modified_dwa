#include "platform_teleop_alg_node.h"

#include <wiimote/State.h>

PlatformTeleopAlgNode::PlatformTeleopAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<PlatformTeleopAlgorithm>()
{
  //init class attributes if necessary
  loop_rate_ = 10;//in [Hz]
  this->human_is_alive_=false;
  this->vt_max            = 0.5;
  this->vr_max            = 0.5;
  this->trans_speed_scale = 0.1;
  this->rot_speed_scale   = 0.1;
  this->cancel_goal       = false;

  // [init publishers]
  this->cancel_goal_publisher_ = this->public_node_handle_.advertise<actionlib_msgs::GoalID>("cancel_goal", 1);
  this->cmd_vel_publisher_ = this->public_node_handle_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  
  // [init subscribers]
  this->joy_subscriber_    = this->public_node_handle_.subscribe("joy", 1, &PlatformTeleopAlgNode::joy_callback, this);
  pthread_mutex_init(&this->joy_mutex_,NULL);

  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

PlatformTeleopAlgNode::~PlatformTeleopAlgNode(void)
{
  // [free dynamic memory]
}

void PlatformTeleopAlgNode::mainNodeThread(void)
{
  this->alg_.lock();
  this->update_joy_watchdog();
  if(this->human_is_alive_)
  {
    if(this->joy_watchdog_active())
    {
      this->human_is_alive_ = false;
      ROS_ERROR("PlatformTeleopAlgNode::mainNodeThread: joy_watchdog timeout!");
      this->twist_msg.linear.x=0.0;
      this->twist_msg.angular.z=0.0;
      this->cmd_vel_publisher_.publish(this->twist_msg);
      this->trans_speed_scale=this->config.translation_increment_step;
      this->rot_speed_scale=this->config.rotation_increment_step;
    }
    else
      this->cmd_vel_publisher_.publish(this->twist_msg);
  }
  this->alg_.unlock();

  // [fill msg structures]
  // Initialize the topic message structure
  //this->cancel_goal_GoalID_msg_.data = my_var;


  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]


}

/*  [subscriber callbacks] */
void PlatformTeleopAlgNode::joy_callback(const sensor_msgs::Joy::ConstPtr& msg) 
{
  this->alg_.lock();
  static std::vector<int> last_buttons=msg->buttons;
  std::vector<int> current_buttons = msg->buttons;
  std::vector<float> current_axes = msg->axes;
  
  this->reset_joy_watchdog();
  
  if(/*this->config.joy_type==0 &&*/ (msg->axes.size()==29 && msg->buttons.size()==17) || (msg->axes.size()==27 && msg->buttons.size()==19) ) //ps3 bluetooth or usb-cable
    this->usePs3(current_buttons, last_buttons, current_axes);
  else if(/*this->config.joy_type==1 &&*/ msg->axes.size()==3 && msg->buttons.size()==11) //wiimote number of axes and buttons
    this->useWii(current_buttons, last_buttons);
  else
    ROS_ERROR("PlatformTeleopAlgNode::joy_callback: unrecognized joy msg (%ld axes, %ld buttons)", msg->axes.size(), msg->buttons.size());
  
  last_buttons=current_buttons;
  
  this->alg_.unlock();
}

void PlatformTeleopAlgNode::joy_mutex_enter(void) 
{ 
  pthread_mutex_lock(&this->joy_mutex_); 
} 

void PlatformTeleopAlgNode::joy_mutex_exit(void) 
{ 
  pthread_mutex_unlock(&this->joy_mutex_); 
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void PlatformTeleopAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->vt_max            = config.max_translation_speed;
  this->vr_max            = config.max_rotation_speed;
  this->trans_speed_scale = config.translation_increment_step;
  this->rot_speed_scale   = config.rotation_increment_step;
  this->cancel_goal       = config.cancel_goal;
  this->config            = config;
  this->alg_.unlock();
}

void PlatformTeleopAlgNode::addNodeDiagnostics(void)
{
}

void PlatformTeleopAlgNode::useWii(std::vector<int> current_buttons, std::vector<int> last_buttons)
{
  if(current_buttons[wiimote::State::MSG_BTN_B]==0)
  {
    this->human_is_alive_=false;
    this->twist_msg.linear.x=0.0;
    this->twist_msg.angular.z=0.0;
    if(last_buttons[wiimote::State::MSG_BTN_B]==1)
    {
      ROS_INFO("Released dead man button. Stop!");
      this->cmd_vel_publisher_.publish(this->twist_msg);
    }
  }
  else
  {
    this->human_is_alive_=true;
    for(unsigned int i=0; i<current_buttons.size(); i++)
    {
      if(current_buttons[i]==1 && last_buttons[i]==0)
        this->useWiiButton(i);
    }
  }
}

void PlatformTeleopAlgNode::useWiiButton(const unsigned int & index)
{
  double vt_step = this->config.translation_increment_step;
  double vr_step = this->config.rotation_increment_step;

  switch(index)
  {
    case wiimote::State::MSG_BTN_B:
      //already considered (dead_man button)
      break;

    case wiimote::State::MSG_BTN_A:
      //ROS_INFO("A:stop!");
      this->twist_msg.linear.x=0.0;
      this->twist_msg.angular.z=0.0;
      ROS_INFO("Pressed A button. Stop!");
      this->cmd_vel_publisher_.publish(this->twist_msg);
      if(this->cancel_goal)
      {
        // Uncomment the following line to publish the topic message
        this->cancel_goal_publisher_.publish(this->cancel_goal_GoalID_msg_);
      }
      break;

    case wiimote::State::MSG_BTN_LEFT:
      //ROS_INFO("left!");
      this->twist_msg.angular.z += vr_step;
      if(this->twist_msg.angular.z > this->vr_max)
        this->twist_msg.angular.z = this->vr_max;
      break;

    case wiimote::State::MSG_BTN_RIGHT:
      //ROS_INFO("right!");
      this->twist_msg.angular.z -= vr_step;
      if(this->twist_msg.angular.z < -this->vr_max)
        this->twist_msg.angular.z = -this->vr_max;
      break;

    case wiimote::State::MSG_BTN_UP:
      //ROS_INFO("forward!");
      this->twist_msg.linear.x += vt_step;
      if(this->twist_msg.linear.x > this->vt_max)
        this->twist_msg.linear.x = this->vt_max;
      break;

    case wiimote::State::MSG_BTN_DOWN:
      //ROS_INFO("backward!");
      this->twist_msg.linear.x -= vt_step;
      if(this->twist_msg.linear.x < -this->vt_max)
        this->twist_msg.linear.x = -this->vt_max;
      break;

    default:
      //ROS_INFO("non defined button");
      break;
  }
}

void PlatformTeleopAlgNode::usePs3(std::vector<int> current_buttons, std::vector<int> last_buttons, std::vector<float> current_axes)
{
  if(current_buttons[BUTTON_DEAD_MAN]==0)
  {
    this->human_is_alive_=false;
    this->twist_msg.linear.x=0.0;
    this->twist_msg.angular.z=0.0;
    if(last_buttons[BUTTON_DEAD_MAN]==1)
    {
      ROS_INFO("Released dead man button. Stop!");
      this->cmd_vel_publisher_.publish(this->twist_msg);
      this->trans_speed_scale=this->config.translation_increment_step;
      this->rot_speed_scale=this->config.rotation_increment_step;
    }
  }
  else
  {
    this->human_is_alive_=true;
    for(unsigned int i=0; i<current_buttons.size(); i++)
    {
      if(current_buttons[i]==1 && last_buttons[i]==0)
        this->usePs3Button(i);
    }
    double vt = this->trans_speed_scale*this->vt_max*current_axes[AXIS_TRANS_FORWARD];
    double vr = this->rot_speed_scale*this->vr_max*current_axes[AXIS_ROT_LEFTWARD];
    this->twist_msg.linear.x  = vt;
    this->twist_msg.angular.z = vr;
  }
}

void PlatformTeleopAlgNode::usePs3Button(const unsigned int & index)
{
  switch(index)
  {
    case BUTTON_DEAD_MAN:
      //already considered
      break;

    case BUTTON_TRANS_SPEED_UP:
      this->trans_speed_scale+=this->config.translation_increment_step;
      if(this->trans_speed_scale>1.0)
        this->trans_speed_scale=1.0;
      else
        ROS_INFO("Setting max translation speed to %.2f (%.0f%% of max %.2f)",
                 this->trans_speed_scale*this->vt_max,
                 this->trans_speed_scale*100.0,
                 this->vt_max);
      break;

    case BUTTON_TRANS_SPEED_DOWN:
      this->trans_speed_scale-=this->config.translation_increment_step;
      if(this->trans_speed_scale<0.0)
        this->trans_speed_scale=0.0;
      else
        ROS_INFO("Setting max translation speed to %.2f (%.0f%% of max %.2f)",
                 this->trans_speed_scale*this->vt_max,
                 this->trans_speed_scale*100.0,
                 this->vt_max);
      break;

    case BUTTON_ROT_SPEED_UP:
      this->rot_speed_scale+=this->config.rotation_increment_step;
      if(this->rot_speed_scale>1.0)
        this->rot_speed_scale=1.0;
      else
        ROS_INFO("Setting max rotation speed to %.2f (%.0f%% of max %.2f)",
                 this->rot_speed_scale*this->vr_max,
                 this->rot_speed_scale*100.0,
                 this->vr_max);
      break;

    case BUTTON_ROT_SPEED_DOWN:
      this->rot_speed_scale-=this->config.rotation_increment_step;
      if(this->rot_speed_scale<0.0)
        this->rot_speed_scale=0.0;
      else
        ROS_INFO("Setting max rotation speed to %.2f (%.0f%% of max %.2f)",
                 this->rot_speed_scale*this->vr_max,
                 this->rot_speed_scale*100.0,
                 this->vr_max);
      break;

    case BUTTON_CANCEL_GOAL: 
      if(this->cancel_goal)
      {
        // Uncomment the following line to publish the topic message
        this->cancel_goal_publisher_.publish(this->cancel_goal_GoalID_msg_);
      }
      break;
    default:
      //ROS_INFO("non defined button");
      break;
  }
}

void PlatformTeleopAlgNode::reset_joy_watchdog(void)
{
  this->joy_watchdog_access.enter();
  this->joy_watchdog_duration=ros::Duration(this->config.joy_watchdog_time);
  this->joy_watchdog_access.exit();
}

bool PlatformTeleopAlgNode::joy_watchdog_active(void)
{
  this->joy_watchdog_access.enter();
  if(this->joy_watchdog_duration.toSec()<=0.0)
  {
    this->joy_watchdog_access.exit();
    return true;
  }
  else
  {
    this->joy_watchdog_access.exit();
    return false;
  }
}

void PlatformTeleopAlgNode::update_joy_watchdog(void)
{
  static ros::Time start_time=ros::Time::now();
  ros::Time current_time=ros::Time::now();

  this->joy_watchdog_access.enter();
  this->joy_watchdog_duration-=(current_time-start_time);
  start_time=current_time;
  this->joy_watchdog_access.exit();
}


/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<PlatformTeleopAlgNode>(argc, argv, "platform_teleop_alg_node");
}
