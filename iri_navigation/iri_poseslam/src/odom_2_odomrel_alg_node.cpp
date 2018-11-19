#include "odom_2_odomrel_alg_node.h"

Odom2OdomrelAlgNode::Odom2OdomrelAlgNode(void) :
algorithm_base::IriBaseAlgorithm<Odom2OdomrelAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]
  last_pose_(0) = 0;
  last_pose_(1) = 0;
  last_pose_(2) = 0;
  public_node_handle_.param<double>("covariance_factor", covariance_factor_, 0.05);
  ROS_DEBUG("ODOM 2 ODOM_REL: Config updated");

  // [init publishers]
  this->odom_rel_publisher_ = this->public_node_handle_.advertise<nav_msgs::Odometry>("odom_rel", 10);

  // [init subscribers]
  this->odom_subscriber_ = this->public_node_handle_.subscribe("odom", 10, &Odom2OdomrelAlgNode::odom_callback, this);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

Odom2OdomrelAlgNode::~Odom2OdomrelAlgNode(void)
{
  // [free dynamic memory]
}

void Odom2OdomrelAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]
  
  // [publish messages]
}

/*  [subscriber callbacks] */
void Odom2OdomrelAlgNode::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_DEBUG("ODOM 2 ODOM_REL New absolute odometry Message Received");
  //ROS_INFO_STREAM(msg->header.covariance);
  Vector3d new_pose = pose_2_vector(msg->pose.pose);
  Vector3d odomrel;

  // FIRST ODOMETRY RELATIVE DISPLACEMENT
  if (last_pose_(0) == 0 && last_pose_(1) == 0 && last_pose_(2) == 0)
    odomrel = last_pose_;
  // RELATIVE DISPLACEMENT
  else
  {
    Matrix3d R = rotation_matrix(-last_pose_(2));
    odomrel = R * (new_pose - last_pose_);
    if (msg->header.seq - last_odom_.header.seq > 1)
      ROS_WARN("ODOM 2 ODOM_REL: Not consecutive odometries!!");
  }
  
  if (odomrel(0) > 1e2 || odomrel(1) > 1e2 || odomrel(2) > 1e2)
  {
      ROS_WARN("ODOM 2 ODOM_REL: Huge relative odometry..!!");
      std::cout << "last pose: " << last_pose_.transpose() << std::endl;
      std::cout << "new pose: " << new_pose.transpose() << std::endl;
      std::cout << "odom rel: " << odomrel.transpose() << std::endl;
  }

  if (std::isnan(odomrel(0)) || std::isnan(odomrel(1)) || std::isnan(odomrel(2)))
  {
      ROS_WARN("ODOM 2 ODOM_REL: NaN in odometry..!!");
      std::cout << "last pose: " << last_pose_.transpose() << std::endl;
      std::cout << "new pose: " << new_pose.transpose() << std::endl;
      std::cout << "odom rel: " << odomrel.transpose() << std::endl;
  }
  else
  {
      // COVARIANCE
      Matrix3d cov = MatrixXd::Zero(3,3);
      for (uint i = 0; i < 3; i++)
        cov(i, i) = std::min(covariance_factor_ * covariance_factor_ * odomrel(i) * odomrel(i), 1e2);

      if ((cov.array() == 100).any())
      {
          ROS_WARN("ODOM 2 ODOM_REL: Odometry took maximum value");
          std::cout << "last pose: " << last_pose_.transpose() << std::endl;
          std::cout << "new pose: " << new_pose.transpose() << std::endl;
          std::cout << "odom rel: " << odomrel.transpose() << std::endl;
          std::cout << "covariance_factor_: " << covariance_factor_ << std::endl;
      }

      // PUBLISH RELATIVE ODOMETRY
      nav_msgs::Odometry new_odomrel;
      new_odomrel.pose = eigen_2_posewithcovariance(odomrel,cov);
      new_odomrel.header = msg->header;
      odom_rel_publisher_.publish(new_odomrel);

      // STORE NEW POSE
      last_pose_ = new_pose;
      last_odom_ = *msg;

      ROS_DEBUG("ODOM 2 ODOM_REL New relative odometry published");
  }
}

/*  [service callbacks] */


Matrix3d Odom2OdomrelAlgNode::rotation_matrix(const double &alpha) const
{
  Matrix3d rot = MatrixXd::Identity(3,3);
  
  rot(0,0) = cos(alpha);
  rot(0,1) = -sin(alpha);
  rot(1,0) = sin(alpha);
  rot(1,1) = cos(alpha);
  
  return rot;
}

Matrix3d Odom2OdomrelAlgNode::covariance_2_matrix(const geometry_msgs::PoseWithCovariance &pose) const
{
  Matrix3d cov;
  
  cov(0,0) = pose.covariance[0];
  cov(0,1) = pose.covariance[1];
  cov(0,2) = pose.covariance[5];
  cov(1,0) = pose.covariance[6];
  cov(1,1) = pose.covariance[7];
  cov(1,2) = pose.covariance[11];
  cov(2,0) = pose.covariance[30];
  cov(2,1) = pose.covariance[31];
  cov(2,2) = pose.covariance[35];
  
  return cov;
}

Vector3d Odom2OdomrelAlgNode::pose_2_vector(const geometry_msgs::Pose &pose) const
{
  Vector3d p;
  
  p(0) = pose.position.x;
  p(1) = pose.position.y;
  p(2) = tf::getYaw(pose.orientation);
  
  return p;
}

geometry_msgs::PoseWithCovariance Odom2OdomrelAlgNode::eigen_2_posewithcovariance(const Vector3d &p, const Matrix3d &cov) const
{
  geometry_msgs::PoseWithCovariance pose;
  
  pose.pose.position.x = p(0);
  pose.pose.position.y = p(1);
  pose.pose.position.z = 0;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(p(2));
  
  pose.covariance[0]  = cov(0,0);
  pose.covariance[1]  = cov(0,1);
  pose.covariance[5]  = cov(0,2);
  pose.covariance[6]  = cov(1,0);
  pose.covariance[7]  = cov(1,1);
  pose.covariance[11] = cov(1,2);
  pose.covariance[30] = cov(2,0);
  pose.covariance[31] = cov(2,1);
  pose.covariance[35] = cov(2,2);
  
  return pose;
}

/*  [action callbacks] */

/*  [action requests] */

void Odom2OdomrelAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->alg_.unlock();
}

void Odom2OdomrelAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<Odom2OdomrelAlgNode>(argc, argv, "scans_2_odom_alg_node");
}
