#include "safe_cmd_alg_node.h"

SafeCmdAlgNode::SafeCmdAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<SafeCmdAlgorithm>(),
  collision_time_(1),
  min_dist_(0.4),
  max_vel_front_(0.0),
  max_vel_rear_(0.0),
  limit_vel_front_(0.0),
  limit_vel_rear_(0.0),
  front_laser_received_(false),
  rear_laser_received_(false),
  new_cmd_vel(false),
  tf_listener_(ros::Duration(10.f))
{
  //init class attributes if necessary
  loop_rate_ = 30;//in [Hz]

  // [init publishers]
  cmd_vel_safe_publisher_ = public_node_handle_.advertise<geometry_msgs::Twist>("cmd_vel_safe", 100);
  
  // [init subscribers]
  this->joy_subscriber_ = this->private_node_handle_.subscribe("joy", 1, &SafeCmdAlgNode::joy_callback, this);
  pthread_mutex_init(&this->joy_mutex_,NULL);

  cmd_vel_subscriber_     = public_node_handle_.subscribe("cmd_vel", 100, &SafeCmdAlgNode::cmd_vel_callback,this);
  rear_laser_subscriber_  = public_node_handle_.subscribe("rear_laser", 100, &SafeCmdAlgNode::rear_laser_callback, this);
  front_laser_subscriber_ = public_node_handle_.subscribe("front_laser", 100, &SafeCmdAlgNode::front_laser_callback, this);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
  
  this->config.front_laser_watchdog_time = 1.0;
  this->config.rear_laser_watchdog_time = 1.0;
  
  this->reset_front_laser_watchdog();
  this->reset_rear_laser_watchdog();
}

SafeCmdAlgNode::~SafeCmdAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->joy_mutex_);
}

void SafeCmdAlgNode::mainNodeThread(void)
{
  this->alg_.lock();
  
  if(!this->config.unsafe)
  {
    if(this->config.mode==1)
    {
      if(this->front_laser_received_ || this->rear_laser_received_)
      {
        std::vector<geometry_msgs::Point32> points;
        
        try
        {
          float cutoff=1.5;
          sensor_msgs::PointCloud cloud_front;
          if(this->front_laser_received_)
          {
            this->laser_projector_.transformLaserScanToPointCloud(this->config.base_frame, this->front_laser_scan, cloud_front, tf_listener_, cutoff);
            this->front_laser_received_=false;
          }
          
          sensor_msgs::PointCloud cloud_rear;
          if(this->rear_laser_received_)
          {
            this->laser_projector_.transformLaserScanToPointCloud(this->config.base_frame, this->rear_laser_scan, cloud_rear, tf_listener_, cutoff);
            this->rear_laser_received_=false;
          }
          
          points =  cloud_front.points;
          points.insert(points.end(), cloud_rear.points.begin(), cloud_rear.points.end());
          
        }
        catch (tf::TransformException &ex)
        {
          ROS_ERROR_DELAYED_THROTTLE(5,"SafeCmdAlgNode:: %s",ex.what());
        }
        

        this->compute_max_velocities_(points, max_vel_front_, max_vel_rear_);
        
        max_vel_front_ = std::min(max_vel_front_, limit_vel_front_);
        max_vel_rear_  = std::max(max_vel_rear_,  limit_vel_rear_);

        //ROS_INFO("front=%f, rear=%f",max_vel_front_, max_vel_rear_);
      
      }
    }
    
    //ROS_INFO("max_vel_front = %f, max_vel_rear = %f", max_vel_front_, max_vel_rear_);
    
    this->update_front_laser_watchdog();
    this->update_rear_laser_watchdog();
    if(this->front_laser_watchdog_active() || this->rear_laser_watchdog_active())
    {
      if(this->front_laser_watchdog_active())
      {
        ROS_ERROR_DELAYED_THROTTLE(5,"SafeCmdAlgNode::mainNodeThread: Front laser watchdog timeout!");
        Twist_msg_.linear.x = 0.0;
      }
      if(this->rear_laser_watchdog_active())
      {
        ROS_ERROR_DELAYED_THROTTLE(5,"SafeCmdAlgNode::mainNodeThread: Rear laser watchdog timeout!");
        Twist_msg_.linear.x = 0.0;
      }
    }
    else
    {
      if(Twist_msg_.linear.x > max_vel_front_)
      {
        //ROS_INFO_STREAM_THROTTLE(1,"SafeCmdAlgNode::mainNodeThread: Reducing forward velocity from "<< Twist_msg_.linear.x << " to "<<fabs(max_vel_front_));
        Twist_msg_.linear.x = max_vel_front_;
        if(max_vel_front_==0)
          Twist_msg_.angular.z = 0;
      }

      if(Twist_msg_.linear.x < max_vel_rear_)
      {
        //ROS_INFO_STREAM_THROTTLE(1,"SafeCmdAlgNode::mainNodeThread: Reducing backward velocity from "<< Twist_msg_.linear.x << " to "<<-fabs(max_vel_rear_));
        Twist_msg_.linear.x = max_vel_rear_;
        if(max_vel_rear_==0)
          Twist_msg_.angular.z = 0;
      }
    }
  }

  if(this->new_cmd_vel)
  {
    cmd_vel_safe_publisher_.publish(Twist_msg_);
    this->new_cmd_vel=false;
  }

  // [fill msg structures]

  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  
  this->alg_.unlock();
}

/*  [subscriber callbacks] */
void SafeCmdAlgNode::joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
  //ROS_INFO("SafeCmdAlgNode::joy_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  this->alg_.lock();
  //this->joy_mutex_enter();
  static bool first=true;
  if(first)
  {
    this->joy_previous_buttons.resize(msg->buttons.size());
    first=false;
  }
  
  if(msg->buttons[this->config.unsafe_joy_button_id]==1) // && this->joy_previous_buttons[this->config.unsafe_joy_button_id]==0)
  {
    ROS_WARN_STREAM_THROTTLE(1, "SafeCmdAlgNode: joy enabled unsafe mode!");
    this->config.unsafe = true; //!this->config.unsafe;
  }
  else
    this->config.unsafe = false;

  this->joy_previous_buttons=msg->buttons;
  this->alg_.unlock();
  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->joy_mutex_exit();
}

void SafeCmdAlgNode::joy_mutex_enter(void)
{
  pthread_mutex_lock(&this->joy_mutex_);
}

void SafeCmdAlgNode::joy_mutex_exit(void)
{
  pthread_mutex_unlock(&this->joy_mutex_);
}

void SafeCmdAlgNode::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg) 
{ 
  //ROS_INFO("SafeCmdAlgNode::cmd_vel_callback: New Message Received"); 
  this->alg_.lock(); 
  //cmd_vel_mutex_.enter();

  Twist_msg_=*msg;
  this->new_cmd_vel=true;

  //cmd_vel_mutex_.exit(); 
  this->alg_.unlock(); 
}
void SafeCmdAlgNode::rear_laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{ 
  //ROS_INFO("SafeCmdAlgNode::rear_laser_callback: New Message Received"); 
  this->alg_.lock(); 
  //rear_laser_mutex_.enter(); 

  this->reset_rear_laser_watchdog();
  this->rear_laser_scan = *msg;
  
  if(this->config.mode == 0)
    this->max_vel_rear_ = std::max(-compute_max_velocity_(msg),limit_vel_rear_);
  this->rear_laser_received_ = true;
  //ROS_INFO("Max vel r: %f",max_vel_rear_);

  //rear_laser_mutex_.exit(); 
  this->alg_.unlock(); 

}
void SafeCmdAlgNode::front_laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{ 
  //ROS_INFO("SafeCmdAlgNode::front_laser_callback: New Message Received"); 
  this->alg_.lock(); 
  //front_laser_mutex_.enter();

  this->reset_front_laser_watchdog();
  this->front_laser_scan = *msg;
  
  if(this->config.mode == 0)
    this->max_vel_front_ = std::min(compute_max_velocity_(msg),limit_vel_front_);
  this->front_laser_received_ = true;
  //ROS_INFO("Max vel f: %f",max_vel_front_);

  //front_laser_mutex_.exit(); 
  this->alg_.unlock(); 
  
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void SafeCmdAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  collision_time_  = config.collision_time;
  min_dist_        = config.min_dist;
  limit_vel_front_ = config.limit_vel_front;
  limit_vel_rear_  = config.limit_vel_rear;
  this->config=config;

  this->alg_.unlock();
}

void SafeCmdAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<SafeCmdAlgNode>(argc, argv, "safe_cmd_alg_node");
}

bool min_test_(float i, float j)
{ 
  if(i<=0.005)
    return false;
  if(j<=0.005)
    return true;
  return i<j;
}

float SafeCmdAlgNode::compute_max_velocity_(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  float min_range    = *std::min_element( scan->ranges.begin(), scan->ranges.end(),min_test_);
  int   min_pos      = distance(scan->ranges.begin(),std::min_element( scan->ranges.begin(), scan->ranges.end() ));
  float max_velocity = 0.0;
  
  //float x = scan->ranges[min_pos]*cos(scan->angle_min + min_pos*scan->angle_increment);
  //float y = scan->ranges[min_pos]*sin(scan->angle_min + min_pos*scan->angle_increment);

  if (min_range >= min_dist_)
    max_velocity = min_range / collision_time_;

  //ROS_INFO_STREAM("SafeCmdAlgNode::compute_max_velocity_: min_range=" << min_range << ", max_velocity=" << max_velocity);

  return max_velocity;
}

void SafeCmdAlgNode::compute_max_velocities_(const std::vector<geometry_msgs::Point32> points, float & max_vel_front, float & max_vel_rear)
{
  float min_range_front=10.0;
  float min_range_rear =10.0;
  for(unsigned int i=0; i<points.size(); i++)
  {
    float x = points[i].x;
    float y = points[i].y;
    float range = sqrt(pow(x,2)+pow(y,2));
    if(x > 0)
    {
      if(range<min_range_front)
        min_range_front=range;
    }
    else if(x < 0)
    {
      if(range<min_range_rear)
        min_range_rear=range;
    }
  }
  
  if(min_range_front > min_dist_)
    max_vel_front = min_range_front / collision_time_;
  else
    max_vel_front = 0.0;

  if(min_range_rear > min_dist_)
    max_vel_rear = - min_range_rear / collision_time_;
  else
    max_vel_rear = 0.0;
}

void SafeCmdAlgNode::reset_front_laser_watchdog(void)
{
  this->front_laser_watchdog_access.enter();
  this->front_laser_watchdog_duration=ros::Duration(this->config.front_laser_watchdog_time);
  this->front_laser_watchdog_access.exit();
}

bool SafeCmdAlgNode::front_laser_watchdog_active(void)
{
  this->front_laser_watchdog_access.enter();
  if(this->front_laser_watchdog_duration.toSec()<=0.0)
  {
    this->front_laser_watchdog_access.exit();
    return true;
  }
  else
  {
    this->front_laser_watchdog_access.exit();
    return false;
  }
}

void SafeCmdAlgNode::update_front_laser_watchdog(void)
{
  static ros::Time start_time=ros::Time::now();
  ros::Time current_time=ros::Time::now();

  this->front_laser_watchdog_access.enter();
  this->front_laser_watchdog_duration-=(current_time-start_time);
  start_time=current_time;
  this->front_laser_watchdog_access.exit();
}

void SafeCmdAlgNode::reset_rear_laser_watchdog(void)
{
  this->rear_laser_watchdog_access.enter();
  this->rear_laser_watchdog_duration=ros::Duration(this->config.rear_laser_watchdog_time);
  this->rear_laser_watchdog_access.exit();
}

bool SafeCmdAlgNode::rear_laser_watchdog_active(void)
{
  this->rear_laser_watchdog_access.enter();
  if(this->rear_laser_watchdog_duration.toSec()<=0.0)
  {
    this->rear_laser_watchdog_access.exit();
    return true;
  }
  else
  {
    this->rear_laser_watchdog_access.exit();
    return false;
  }
}

void SafeCmdAlgNode::update_rear_laser_watchdog(void)
{
  static ros::Time start_time=ros::Time::now();
  ros::Time current_time=ros::Time::now();

  this->rear_laser_watchdog_access.enter();
  this->rear_laser_watchdog_duration-=(current_time-start_time);
  start_time=current_time;
  this->rear_laser_watchdog_access.exit();
}

