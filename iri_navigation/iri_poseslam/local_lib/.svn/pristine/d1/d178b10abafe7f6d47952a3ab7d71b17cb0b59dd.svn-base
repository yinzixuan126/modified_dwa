#include "pose.h"

// 2D POSE CONSTRUCTORS
// Construction with 3 double x,y,theta
CPose::CPose(const double& x,const double& y,const double& th)
{
  is2D=true;
  Pose2D.set_pose(x,y,th);
}

// Construction with array of 3 double x,y,theta
CPose::CPose(const double xyth_new[3])
{
  is2D=true;
  Pose2D.set_pose(xyth_new[0],xyth_new[1],xyth_new[2]);
}

// Construction of 2d pose from vector3f
CPose::CPose(const Vector3d& xytheta)
{
  is2D=true;
  Pose2D.set_pose(xytheta(0),xytheta(1),xytheta(2));
}

// Construction of 2d pose coping from other 2d pose
CPose::CPose(const CPose2D& pose_new)
{
  is2D=true;
  Pose2D.set_pose(pose_new);
}

// DESTRUCTOR
CPose::~CPose()
{
}

// GETS
// get vector of pose2D or pose3D
VectorXd CPose::get_vector()  const
{
  VectorXd vect;
  if (is2D) vect=Pose2D.get_vector();
  
  return vect; 
}

// SETS 2D
// set pose from 3 floats x,y,theta
void CPose::set_pose(const double& x,const double& y,const double& th)
{
  Pose2D.set_pose(x,y,th);
}

// set pose from array of 3 double x,y,theta
void CPose::set_pose(const double xyth_new[3])
{
  Pose2D.set_pose(xyth_new);
}

// set pose from vector3f
void CPose::set_pose(const Vector3d& xyth_new)
{
  Pose2D.set_pose(xyth_new);
}

// set pose coping from other 2d pose
void CPose::set_pose(const CPose2D& pose_new)
{
  Pose2D.set_pose(pose_new);
}

// set pose coping from other pose
void CPose::set_pose(const CPose& pose_new)
{
  if (is2D) Pose2D.set_pose(pose_new.Pose2D);
}

// METHODS
// Print values of Pose
void CPose::print_pose() const
{
  if (is2D) Pose2D.print_pose();
}

// Return Pose P2 result of aplying displacement D (in P1 reference) from P1
CPose CPose::relative_2_absolute(const CPose& D) const
{
  CPose P2;
  if (is2D && D.is2D){
    P2 = Pose2D.relative_2_absolute(D.Pose2D);
  }
  return P2;
}

// Return Jp and Jd in a struct J, the jacobians of displacement D
Jacobians CPose::relative_2_absolute_jacobian(const CPose& D) const
{
  Jacobians J;
  if (is2D && D.is2D){
    J = Pose2D.relative_2_absolute_jacobian(D.Pose2D);
  }
  return J;
}

// Returns the relative displacement, D, from the pose P1 (point from calling function) to pose P2 expresed in the frame of P1.
CPose CPose::absolute_2_relative(const CPose& P2) const
{
  CPose D;
  if (is2D && P2.is2D){
    D= Pose2D.absolute_2_relative(P2.Pose2D);
  }
  return D;
}

// Returns the Jacobians of the Absolute2Relative function with respect its parameteres evaluated at P1 (calling function) and P2.
Jacobians CPose::absolute_2_relative_jacobian(const CPose& P2) const
{
  Jacobians J;
  if (is2D && P2.is2D){
    J = Pose2D.absolute_2_relative_jacobian(P2.Pose2D);
  }
  return J;
}
