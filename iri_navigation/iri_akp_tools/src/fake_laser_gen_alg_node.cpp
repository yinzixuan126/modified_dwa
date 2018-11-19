#include "fake_laser_gen_alg_node.h"

FakeLaserGenAlgNode::FakeLaserGenAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<FakeLaserGenAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  this->scan_publisher_ = this->public_node_handle_.advertise<sensor_msgs::LaserScan>("scan", 10);
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
  this->init();
}

FakeLaserGenAlgNode::~FakeLaserGenAlgNode(void)
{
  // [free dynamic memory]
}

void FakeLaserGenAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  // Initialize the topic message structure
  this->LaserScan_msg_.header.stamp = ros::Time::now();

  
  // [fill srv structure and make request to the server]
  // [fill action structure and make request to the action server]

  // [publish messages]
  this->scan_publisher_.publish(this->LaserScan_msg_);

}


void FakeLaserGenAlgNode::init()
{
/*
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  float32 angle_min
  float32 angle_max
  float32 angle_increment
  float32 time_increment
  float32 scan_time
  float32 range_min
  float32 range_max
  float32[] ranges
*/

  LaserScan_msg_.header.frame_id = "/map";
  LaserScan_msg_.angle_min = -2*1.56643295288;
  LaserScan_msg_.angle_max = 2*1.56643295288;
  LaserScan_msg_.angle_increment = 0.00436332309619;
  LaserScan_msg_.time_increment = 9.99999974738e-05;
  LaserScan_msg_.scan_time = 0.0250000003725;
  LaserScan_msg_.range_min = 0.0;
  LaserScan_msg_.range_max = 60.0;

}


/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void FakeLaserGenAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  ROS_INFO("updating config");
  laser_mode_ = (FakeLaserGenAlgNode::laser_mode) config.laser_mode;
  //plot fixed map
  LaserScan_msg_.ranges.clear();
  LaserScan_msg_.ranges.resize(1436,59.9);
  switch(laser_mode_)
  {
    case FakeLaserGenAlgNode::one_obstacle :
        ROS_INFO("one obstacle");
        LaserScan_msg_.ranges[718]= 1.0;
      break;
    case FakeLaserGenAlgNode::two_obstacles :
      ROS_INFO("two obstacles");
       LaserScan_msg_.ranges[360]= 0.8;
       LaserScan_msg_.ranges[1030]= 0.8;
      break;
    case FakeLaserGenAlgNode::n_obstacles :
      ROS_INFO("n obstacles");
       LaserScan_msg_.ranges[100]= 2.0;
       LaserScan_msg_.ranges[100+40]= 2.0;
       LaserScan_msg_.ranges[340]= 2.0;
       LaserScan_msg_.ranges[340+40]= 2.0;
       LaserScan_msg_.ranges[580]= 2.0;
        LaserScan_msg_.ranges[580+40]= 2.0;
       LaserScan_msg_.ranges[820]= 2.0;
        LaserScan_msg_.ranges[820+40]= 2.0;
       LaserScan_msg_.ranges[1060]= 2.0;
      LaserScan_msg_.ranges[1060+40]= 2.0;
       LaserScan_msg_.ranges[1300]= 2.0;
        LaserScan_msg_.ranges[1300+40]= 2.0;
      break;
    case FakeLaserGenAlgNode::n_obstacles_2 :
      ROS_INFO("n obstacles 2");
       LaserScan_msg_.ranges[100]= 2.0;
       LaserScan_msg_.ranges[100+40]= 2.0;
       LaserScan_msg_.ranges[340]= 2.0;
       LaserScan_msg_.ranges[340+40]= 2.0;
       LaserScan_msg_.ranges[580]= 2.0;
        LaserScan_msg_.ranges[580+40]= 2.0;
       LaserScan_msg_.ranges[820]= 2.0;
        LaserScan_msg_.ranges[820+40]= 2.0;
       LaserScan_msg_.ranges[1060]= 2.0;
      LaserScan_msg_.ranges[1060+40]= 2.0;
       LaserScan_msg_.ranges[1300]= 2.0;
        LaserScan_msg_.ranges[1300+40]= 2.0;
       LaserScan_msg_.ranges[1]= 4.0;
       LaserScan_msg_.ranges[241]= 4.0;
       LaserScan_msg_.ranges[481]= 4.0;
       LaserScan_msg_.ranges[721]= 4.0;
       LaserScan_msg_.ranges[961]= 4.0;
       LaserScan_msg_.ranges[1201]= 4.0;
       LaserScan_msg_.ranges[1+20]= 4.0;
       LaserScan_msg_.ranges[241+20]= 4.0;
       LaserScan_msg_.ranges[481+20]= 4.0;
       LaserScan_msg_.ranges[721+20]= 4.0;
       LaserScan_msg_.ranges[961+20]= 4.0;
       LaserScan_msg_.ranges[1201+20]= 4.0;
      break;
    case FakeLaserGenAlgNode::scene :
      ROS_INFO("scene");
      for (unsigned int n = 20; n<700; ++n)
        LaserScan_msg_.ranges[n] = 1.0 / sin(LaserScan_msg_.angle_increment * (double) n);
      for (unsigned int n = 758; n<900; ++n)
        LaserScan_msg_.ranges[n] = -3.0 / sin(LaserScan_msg_.angle_increment * (double) n);
      for (unsigned int n = 900; n<1040; ++n)
        LaserScan_msg_.ranges[n] = -3.0 / cos(LaserScan_msg_.angle_increment * (double) n);
      for (unsigned int n = 1137; n<1260; ++n)
        LaserScan_msg_.ranges[n] = 3.0 / cos(LaserScan_msg_.angle_increment * (double) n);
      for (unsigned int n = 1260; n<1400; ++n)
        LaserScan_msg_.ranges[n] = -3.0 / sin(LaserScan_msg_.angle_increment * (double) n);
      break;
    case FakeLaserGenAlgNode::scene_2 :
      ROS_INFO("scene");
      for (unsigned int n = 20; n<700; ++n)
        LaserScan_msg_.ranges[n] = 1.0 / sin(LaserScan_msg_.angle_increment * (double) n);
      for (unsigned int n = 738; n<900; ++n)
        LaserScan_msg_.ranges[n] = -1.0 / sin(LaserScan_msg_.angle_increment * (double) n);
      for (unsigned int n = 900; n<1055; ++n)
        LaserScan_msg_.ranges[n] = -1.0 / cos(LaserScan_msg_.angle_increment * (double) n);
      for (unsigned int n = 1107; n<1260; ++n)
        LaserScan_msg_.ranges[n] = 1.0 / cos(LaserScan_msg_.angle_increment * (double) n);
      for (unsigned int n = 1260; n<1400; ++n)
        LaserScan_msg_.ranges[n] = -1.0 / sin(LaserScan_msg_.angle_increment * (double) n);
      break;
    case FakeLaserGenAlgNode::Zero :
    default :
      ROS_INFO("zero");
      for( unsigned int i = 0; i <= 2*718; i++ )
         LaserScan_msg_.ranges[i] = 69.9;
      break;
  }
  this->config_=config;
  this->alg_.unlock();
}

void FakeLaserGenAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<FakeLaserGenAlgNode>(argc, argv, "fake_laser_gen_alg_node");
}
