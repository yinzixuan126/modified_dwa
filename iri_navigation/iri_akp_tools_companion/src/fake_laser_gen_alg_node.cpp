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
  //laser_mode_=FakeLaserGenAlgNode::n_obstacles5;  //caso estrechez en medio del camino. Muy estrecho!
  //laser_mode_=FakeLaserGenAlgNode::n_obstacles3;  //caso estrechez en medio del camino! obstaculos a 2.2305 m entre ellos va con empty_map_whit_obstacles1
  laser_mode_=FakeLaserGenAlgNode::n_obstacles7; //  [ultimo que estaba usando BUENO, datos paper robot2017] caso estrechez en medio del camino!  caso obstaculos a  1.94 m entre ellos. va con empty_map_whit_obstacles2
 // laser_mode_=FakeLaserGenAlgNode::n_obstacles7_robot2017_videos;
  //laser_mode_=FakeLaserGenAlgNode::n_obstacles8; // [ultimo que estaba usando BUENO] caso estrechez en medio del camino!  caso obstaculos a 1.4965 m entre ellos. va con empty_map_whit_obstacles2
  //laser_mode_=FakeLaserGenAlgNode::n_obstacles6;  //caso estrechez en medio del camino!
  //laser_mode_=FakeLaserGenAlgNode::scene_3;  // caso pasillo, esquina. (Ancho)
  //laser_mode_=FakeLaserGenAlgNode::scene_2;  // Estrecho! caso pasillo, esquina.
  //laser_mode_=FakeLaserGenAlgNode::scene_4;  // pasillo esquina, medio ancho, medio estrecho!

  switch(laser_mode_)
  {
    case FakeLaserGenAlgNode::one_obstacle :
    {
        ROS_INFO("one obstacle");
        LaserScan_msg_.ranges[718]= 1.0;
    }
      break;

    case FakeLaserGenAlgNode::two_obstacles :
    {
      ROS_INFO("two obstacles");
       LaserScan_msg_.ranges[360]= 0.8;
       LaserScan_msg_.ranges[1030]= 0.8;
    }
      break;

    case FakeLaserGenAlgNode::n_obstacles :
    {
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
    }
      break;

    case FakeLaserGenAlgNode::n_obstacles_2 :
    {
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
    }
      break;

    case FakeLaserGenAlgNode::scene :
    {
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
    }
      break;
    case FakeLaserGenAlgNode::scene_2 :
    {
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
    }
      break;

    case FakeLaserGenAlgNode::scene_3 :
    {
      ROS_INFO("scene");
      for (unsigned int n = 20; n<700; ++n)
        LaserScan_msg_.ranges[n] = 3.0 / sin(LaserScan_msg_.angle_increment * (double) n);
      for (unsigned int n = 758; n<805; ++n)
        LaserScan_msg_.ranges[n] = -2.0 / sin(LaserScan_msg_.angle_increment * (double) n);
      for (unsigned int n = 805; n<1040; ++n)
        LaserScan_msg_.ranges[n] = -5.0 / cos(LaserScan_msg_.angle_increment * (double) n);
      for (unsigned int n = 1137; n<1260; ++n)
        LaserScan_msg_.ranges[n] = 3.0 / cos(LaserScan_msg_.angle_increment * (double) n);
      for (unsigned int n = 1260; n<1400; ++n)
        LaserScan_msg_.ranges[n] = -3.0 / sin(LaserScan_msg_.angle_increment * (double) n);
    }
      break;

    case FakeLaserGenAlgNode::scene_4 :
    {
      ROS_INFO("scene");
      for (unsigned int n = 20; n<700; ++n)
        LaserScan_msg_.ranges[n] = 1.0 / sin(LaserScan_msg_.angle_increment * (double) n);
      for (unsigned int n = 738; n<900; ++n)
        LaserScan_msg_.ranges[n] = -3.0 / sin(LaserScan_msg_.angle_increment * (double) n);
      for (unsigned int n = 900; n<1055; ++n)
        LaserScan_msg_.ranges[n] = -3.0 / cos(LaserScan_msg_.angle_increment * (double) n);
      for (unsigned int n = 1107; n<1260; ++n)
        LaserScan_msg_.ranges[n] = 3.0 / cos(LaserScan_msg_.angle_increment * (double) n);
      for (unsigned int n = 1260; n<1400; ++n)
        LaserScan_msg_.ranges[n] = -3.0 / sin(LaserScan_msg_.angle_increment * (double) n);
    }
      break;

     case FakeLaserGenAlgNode::corrido_obstacles_companion_2_paths :
    {
       ROS_INFO("corrido_obstacles_companion_2_paths");

      for (unsigned int n = 20; n<700; ++n)
        LaserScan_msg_.ranges[n] = 1.0 / sin(LaserScan_msg_.angle_increment * (double) n);
      for (unsigned int n = 738; n<900; ++n)
        LaserScan_msg_.ranges[n] = -1.0 / sin(LaserScan_msg_.angle_increment * (double) n);
       //LaserScan_msg_.ranges[360]= 1.0;
       //LaserScan_msg_.ranges[695]= 1.0;
      // LaserScan_msg_.ranges[1030]= 1.0;

      // LaserScan_msg_.ranges[360]= 2.0;
      // LaserScan_msg_.ranges[695]= 2.0;
       //LaserScan_msg_.ranges[1030]= 2.0;
    }
      break;

    case FakeLaserGenAlgNode::corrido_obstacles_companion_3_paths :
    {
       ROS_INFO("corrido_obstacles_companion_3_paths");

      for (unsigned int n = 538; n<700; ++n)
        LaserScan_msg_.ranges[n] = 1.0 / sin(LaserScan_msg_.angle_increment * (double) n);
      for (unsigned int n = 738; n<900; ++n)
        LaserScan_msg_.ranges[n] = -1.0 / sin(LaserScan_msg_.angle_increment * (double) n);
    }
      break;

    case FakeLaserGenAlgNode::corridor_obstacles_which_narrows_companion_1_paths :
    {
       ROS_INFO("corridor_obstacles_which_narrows_companion_1_paths");
        int cont=0;
      for (unsigned int n = 0; n<649; ++n){  // +81  => 81/4=20.alg0 => 538+81+30=649
       // LaserScan_msg_.ranges[n] = 1.0 / sin(LaserScan_msg_.angle_increment * (double) n);
        LaserScan_msg_.ranges[n] = (4.0-(0.0046*cont))/ sin(LaserScan_msg_.angle_increment * (double) n);
        cont++;
      }
      //for (unsigned int n = 538; n<649; ++n){  // +81  => 81/4=20.alg0 => 538+81+30=649
       // LaserScan_msg_.ranges[n] = 1.0 / sin(LaserScan_msg_.angle_increment * (double) n);
      //  LaserScan_msg_.ranges[n] = (3.0-(0.027*cont))/ sin(LaserScan_msg_.angle_increment * (double) n);
      //  cont++;
     // }
      for (unsigned int n = 649; n<700; ++n){
        LaserScan_msg_.ranges[n] = 1.0 / sin(LaserScan_msg_.angle_increment * (double) n);
       // LaserScan_msg_.ranges[n] = (1.0 + (((double) n)/2))/ sin(LaserScan_msg_.angle_increment * (double) n);
      }
      for (unsigned int n = 738; n<789; ++n){  // +81  => 81/4=20.alg0 => 738+81-30=839
        LaserScan_msg_.ranges[n] = -1.0 / sin(LaserScan_msg_.angle_increment * (double) n);
       // LaserScan_msg_.ranges[n] = (-1.0 - (((double) n)/2))/ sin(LaserScan_msg_.angle_increment * (double) n);
      }
      cont=0;
      for (unsigned int n =789 ; n<1400; ++n){
        //LaserScan_msg_.ranges[n] = -1.0 / sin(LaserScan_msg_.angle_increment * (double) n);
        LaserScan_msg_.ranges[n] = (-1.0-(0.00046*cont))/ sin(LaserScan_msg_.angle_increment * (double) n);
        cont++;
      }
    }
      break;

    case FakeLaserGenAlgNode::n_obstacles3 : // caso obstaculos a 2.2305 m entre ellos. va con empty_map_whit_obstacles1
    {
      ROS_INFO("n obstacles 3");
      /* LaserScan_msg_.ranges[100]= 4.0;
       LaserScan_msg_.ranges[100+40]= 4.0;
       LaserScan_msg_.ranges[340]= 4.0;
       LaserScan_msg_.ranges[340+40]= 4.0;
       LaserScan_msg_.ranges[580]= 4.0;
        LaserScan_msg_.ranges[580+40]= 4.0;
       LaserScan_msg_.ranges[820]= 4.0;
        LaserScan_msg_.ranges[820+40]= 4.0;
       LaserScan_msg_.ranges[1060]= 4.0;
      LaserScan_msg_.ranges[1060+40]= 4.0;
       LaserScan_msg_.ranges[1300]= 4.0;
        LaserScan_msg_.ranges[1300+40]= 4.0;*/

       LaserScan_msg_.ranges[100]= 7.0;
       LaserScan_msg_.ranges[100+40]= 7.0;
       LaserScan_msg_.ranges[340]= 7.0;
       //LaserScan_msg_.ranges[340+40]= 7.0;
      // LaserScan_msg_.ranges[660]= 13.25;
        LaserScan_msg_.ranges[680-30]= 13.0;
       LaserScan_msg_.ranges[680-25]= 13.25;
        LaserScan_msg_.ranges[680-20]= 13.25;
      LaserScan_msg_.ranges[680-15]= 13.15;
       LaserScan_msg_.ranges[680-10]= 13.15;
        LaserScan_msg_.ranges[680-5]= 13.0;
       LaserScan_msg_.ranges[730-5]= 13.0;
        LaserScan_msg_.ranges[730]= 13.15;
        LaserScan_msg_.ranges[730+5]= 13.15;
       LaserScan_msg_.ranges[730+10]= 13.15;
      LaserScan_msg_.ranges[730+15]= 13.15;
       LaserScan_msg_.ranges[730+20]= 13.15;
        LaserScan_msg_.ranges[1300+18]= 7.0;
    }
    break;


  case FakeLaserGenAlgNode::n_obstacles7 : // caso obstaculos a  1.94 m entre ellos. va con empty_map_whit_obstacles2
    {
      ROS_INFO("n obstacles 3");
      /* LaserScan_msg_.ranges[100]= 4.0;
       LaserScan_msg_.ranges[100+40]= 4.0;
       LaserScan_msg_.ranges[340]= 4.0;
       LaserScan_msg_.ranges[340+40]= 4.0;
       LaserScan_msg_.ranges[580]= 4.0;
        LaserScan_msg_.ranges[580+40]= 4.0;
       LaserScan_msg_.ranges[820]= 4.0;
        LaserScan_msg_.ranges[820+40]= 4.0;
       LaserScan_msg_.ranges[1060]= 4.0;
      LaserScan_msg_.ranges[1060+40]= 4.0;
       LaserScan_msg_.ranges[1300]= 4.0;
        LaserScan_msg_.ranges[1300+40]= 4.0;*/

       //LaserScan_msg_.ranges[100]= 7.0;
       //LaserScan_msg_.ranges[100+40]= 7.0;
       //LaserScan_msg_.ranges[680-35]= 13.45;
      // LaserScan_msg_.ranges[680-30]= 13.45;
      // LaserScan_msg_.ranges[680-25]= 13.40;
        LaserScan_msg_.ranges[680-20]= 13.30;
       LaserScan_msg_.ranges[680-15]= 13.25;
        LaserScan_msg_.ranges[680-10]= 13.25;
      LaserScan_msg_.ranges[680-5]= 13.15;
       LaserScan_msg_.ranges[680]= 13.15;
        LaserScan_msg_.ranges[680+5]= 13.0;
      // LaserScan_msg_.ranges[730]= 13.0;
      //  LaserScan_msg_.ranges[730+5]= 13.15;
      //  LaserScan_msg_.ranges[730+10]= 13.15;
       LaserScan_msg_.ranges[730+15]= 13.15;
      LaserScan_msg_.ranges[730+20]= 13.20;
       LaserScan_msg_.ranges[730+25]= 13.20;
        LaserScan_msg_.ranges[730+30]= 13.25;
      LaserScan_msg_.ranges[730+35]= 13.25;
        LaserScan_msg_.ranges[730+40]= 13.30;
      LaserScan_msg_.ranges[730+45]= 13.30;
    }
    break;


 case FakeLaserGenAlgNode::n_obstacles7_robot2017_videos : // caso obstaculos a  1.94 m entre ellos. va con empty_map_whit_obstacles2
    {
      ROS_INFO("n obstacles 3");
      /* LaserScan_msg_.ranges[100]= 4.0;
       LaserScan_msg_.ranges[100+40]= 4.0;
       LaserScan_msg_.ranges[340]= 4.0;
       LaserScan_msg_.ranges[340+40]= 4.0;
       LaserScan_msg_.ranges[580]= 4.0;
        LaserScan_msg_.ranges[580+40]= 4.0;
       LaserScan_msg_.ranges[820]= 4.0;
        LaserScan_msg_.ranges[820+40]= 4.0;
       LaserScan_msg_.ranges[1060]= 4.0;
      LaserScan_msg_.ranges[1060+40]= 4.0;
       LaserScan_msg_.ranges[1300]= 4.0;
        LaserScan_msg_.ranges[1300+40]= 4.0;*/

       //LaserScan_msg_.ranges[100]= 7.0;
       //LaserScan_msg_.ranges[100+40]= 7.0;
      // LaserScan_msg_.ranges[340]= 7.0;
       //LaserScan_msg_.ranges[340+40]= 7.0;
      // LaserScan_msg_.ranges[660]= 13.25;
       // LaserScan_msg_.ranges[680-20]= 13.0;
       //LaserScan_msg_.ranges[680-15]= 13.25;
        LaserScan_msg_.ranges[680-15]= 13.25;
      LaserScan_msg_.ranges[680-10]= 13.15;
       LaserScan_msg_.ranges[680-5]= 13.15;
        LaserScan_msg_.ranges[680]= 13.0;

       LaserScan_msg_.ranges[730+10]= 13.0;
        LaserScan_msg_.ranges[730+15]= 13.15;
        LaserScan_msg_.ranges[730+20]= 13.15;
       LaserScan_msg_.ranges[730+25]= 13.15;
      //LaserScan_msg_.ranges[730+15]= 13.15;
       //LaserScan_msg_.ranges[730+20]= 13.15;
        //LaserScan_msg_.ranges[1300+18]= 7.0;
    }
    break;


  case FakeLaserGenAlgNode::n_obstacles8 : // caso obstaculos a 1.4965 m entre ellos. va con empty_map_whit_obstacles2
    {
      ROS_INFO("n obstacles 3");
      /* LaserScan_msg_.ranges[100]= 4.0;
       LaserScan_msg_.ranges[100+40]= 4.0;
       LaserScan_msg_.ranges[340]= 4.0;
       LaserScan_msg_.ranges[340+40]= 4.0;
       LaserScan_msg_.ranges[580]= 4.0;
        LaserScan_msg_.ranges[580+40]= 4.0;
       LaserScan_msg_.ranges[820]= 4.0;
        LaserScan_msg_.ranges[820+40]= 4.0;
       LaserScan_msg_.ranges[1060]= 4.0;
      LaserScan_msg_.ranges[1060+40]= 4.0;
       LaserScan_msg_.ranges[1300]= 4.0;
        LaserScan_msg_.ranges[1300+40]= 4.0;*/

       LaserScan_msg_.ranges[100]= 7.0;
       LaserScan_msg_.ranges[100+40]= 7.0;
       LaserScan_msg_.ranges[340]= 7.0;
       //LaserScan_msg_.ranges[340+40]= 7.0;
      // LaserScan_msg_.ranges[660]= 13.25;
        LaserScan_msg_.ranges[680-15]= 13.0;
       LaserScan_msg_.ranges[680-10]= 13.25;
        LaserScan_msg_.ranges[680-5]= 13.25;
      LaserScan_msg_.ranges[680]= 13.15;
       LaserScan_msg_.ranges[680+8]= 13.15;  // 6.5 pasan...
       // LaserScan_msg_.ranges[680+5]= 13.0;
       LaserScan_msg_.ranges[730-3.5]= 13.15;  //4.5  pasan...
        LaserScan_msg_.ranges[730]= 13.15;
        LaserScan_msg_.ranges[730+5]= 13.15;
       LaserScan_msg_.ranges[730+10]= 13.15;
      LaserScan_msg_.ranges[730+15]= 13.15;
       LaserScan_msg_.ranges[730+20]= 13.15;
        LaserScan_msg_.ranges[1300+18]= 7.0;
    }
    break;



 case FakeLaserGenAlgNode::n_obstacles6 :
    {
      ROS_INFO("n obstacles 6");
      /* LaserScan_msg_.ranges[100]= 4.0;
       LaserScan_msg_.ranges[100+40]= 4.0;
       LaserScan_msg_.ranges[340]= 4.0;
       LaserScan_msg_.ranges[340+40]= 4.0;
       LaserScan_msg_.ranges[580]= 4.0;
        LaserScan_msg_.ranges[580+40]= 4.0;
       LaserScan_msg_.ranges[820]= 4.0;
        LaserScan_msg_.ranges[820+40]= 4.0;
       LaserScan_msg_.ranges[1060]= 4.0;
      LaserScan_msg_.ranges[1060+40]= 4.0;
       LaserScan_msg_.ranges[1300]= 4.0;
        LaserScan_msg_.ranges[1300+40]= 4.0;*/

       LaserScan_msg_.ranges[100]= 7.0;
       LaserScan_msg_.ranges[100+40]= 7.0;
       LaserScan_msg_.ranges[340]= 7.0;
       //LaserScan_msg_.ranges[340+40]= 7.0;
      // LaserScan_msg_.ranges[660]= 13.25;
      //  LaserScan_msg_.ranges[660+20]= 13.0;
      // LaserScan_msg_.ranges[760]= 13.0;
      //  LaserScan_msg_.ranges[760+20]= 13.25;
      LaserScan_msg_.ranges[690-5]= 15.15;
       LaserScan_msg_.ranges[690]= 15.15;
        LaserScan_msg_.ranges[690+5]= 15.0;
       LaserScan_msg_.ranges[720]= 15.0;
        LaserScan_msg_.ranges[720+5]= 15.15;
        LaserScan_msg_.ranges[720+10]= 15.15;
      // LaserScan_msg_.ranges[1060]= 7.0;
      LaserScan_msg_.ranges[1060+40]= 7.0;
       LaserScan_msg_.ranges[1300]= 7.0;
        LaserScan_msg_.ranges[1300+40]= 7.0;
    }
    break;

  case FakeLaserGenAlgNode::n_obstacles5 :
    {
      ROS_INFO("n obstacles 3");
      /* LaserScan_msg_.ranges[100]= 4.0;
       LaserScan_msg_.ranges[100+40]= 4.0;
       LaserScan_msg_.ranges[340]= 4.0;
       LaserScan_msg_.ranges[340+40]= 4.0;
       LaserScan_msg_.ranges[580]= 4.0;
        LaserScan_msg_.ranges[580+40]= 4.0;
       LaserScan_msg_.ranges[820]= 4.0;
        LaserScan_msg_.ranges[820+40]= 4.0;
       LaserScan_msg_.ranges[1060]= 4.0;
      LaserScan_msg_.ranges[1060+40]= 4.0;
       LaserScan_msg_.ranges[1300]= 4.0;
        LaserScan_msg_.ranges[1300+40]= 4.0;*/

       LaserScan_msg_.ranges[685-20]= 13.15;
       LaserScan_msg_.ranges[685-15]= 13.15;
       //LaserScan_msg_.ranges[340]= 7.0;
       //LaserScan_msg_.ranges[340+40]= 7.0;
      LaserScan_msg_.ranges[685-10]= 13.15;
      LaserScan_msg_.ranges[685-5]=13.15;
      // LaserScan_msg_.ranges[660]= 13.25;
      //  LaserScan_msg_.ranges[660+20]= 13.0;
      // LaserScan_msg_.ranges[760]= 13.0;
      //  LaserScan_msg_.ranges[760+20]= 13.25;
      // LaserScan_msg_.ranges[685]= 13.15;
       // LaserScan_msg_.ranges[685+5]= 13.0;
       //LaserScan_msg_.ranges[725]= 13.0;
      //  LaserScan_msg_.ranges[725+5]= 13.15;
       LaserScan_msg_.ranges[725+10]= 13.15;
      LaserScan_msg_.ranges[725+15]= 13.15;
      LaserScan_msg_.ranges[725+20]= 13.15;
      LaserScan_msg_.ranges[725+25]= 13.15;
       LaserScan_msg_.ranges[1300]= 7.0;
        LaserScan_msg_.ranges[1300+40]= 7.0;
    }
    break;

    case FakeLaserGenAlgNode::n_obstacles4 :
    {
      ROS_INFO("n obstacles 3");
       LaserScan_msg_.ranges[100]= 4.0;
       LaserScan_msg_.ranges[100+40]= 4.0;
       //LaserScan_msg_.ranges[340]= 4.0;
       //LaserScan_msg_.ranges[340+40]= 4.0;
       // LaserScan_msg_.ranges[580]= 4.5;  //+
        LaserScan_msg_.ranges[580+40]= 4.0;
       LaserScan_msg_.ranges[770]= 4.0;
        LaserScan_msg_.ranges[770+40]= 4.5;
        //LaserScan_msg_.ranges[820]= 4.0;
        //LaserScan_msg_.ranges[820+40]= 4.5;
       LaserScan_msg_.ranges[1060]= 4.0;
      LaserScan_msg_.ranges[1060+40]= 4.0;
       LaserScan_msg_.ranges[1300]= 4.0;
        LaserScan_msg_.ranges[1300+40]= 4.0;
    }
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
