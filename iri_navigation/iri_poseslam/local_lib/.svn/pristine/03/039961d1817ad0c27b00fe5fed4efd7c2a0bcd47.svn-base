#include "pose2D.h"
using namespace Eigen;

// CONSTRUCTORS
// constructor with non parameters
CPose2D::CPose2D()
{
  xytheta(0)=0.0;
  xytheta(1)=0.0;
  xytheta(2)=0.0;
}

// Construction with 3 double x,y,theta
CPose2D::CPose2D(const double& x,const double& y,const double& th)
{
  xytheta(0)=x;
  xytheta(1)=y;
  xytheta(2)=th;
  angle_correction();
}

// Construction with array of 3 double x,y,theta
CPose2D::CPose2D(const double xyth_new[3])
{
  xytheta(0)=xyth_new[0];
  xytheta(1)=xyth_new[1];
  xytheta(2)=xyth_new[2];
  angle_correction();
}

// Construction of 2d pose from vector3f
CPose2D::CPose2D(const Vector3d& xyth_new)
{
  xytheta=xyth_new;
  angle_correction();
}

// Construction of 2d pose coping from other 2d pose
CPose2D::CPose2D(const CPose2D& pose_new)
{
  xytheta=pose_new.xytheta;
  angle_correction();
}
// DESTRUCTOR
CPose2D::~CPose2D() 
{
}

// SETS
// set pose from 3 floats x,y,theta
void CPose2D::set_pose(const double& x,const double& y,const double& th)
{
  xytheta(0)=x;
  xytheta(1)=y;
  xytheta(2)=th;
  angle_correction();
}

// set pose from array of 3 double x,y,theta
void CPose2D::set_pose(const double xyth_new[3])
{
  xytheta(0)=xyth_new[0];
  xytheta(1)=xyth_new[1];
  xytheta(2)=xyth_new[2];
  angle_correction();
}

// set pose from vector3f
void CPose2D::set_pose(const Vector3d& xyth_new)
{
  xytheta=xyth_new;
  angle_correction();
}

// set pose coping from other 2d pose
void CPose2D::set_pose(const CPose2D& pose_new)
{
  xytheta=pose_new.xytheta;
  angle_correction();
}


//GETS
// get vector3d of pose2d
Vector3d CPose2D::get_vector() const
{
  return xytheta;
}

// METHODS

// Put the angle into the (-PI, PI] interval
void CPose2D::angle_correction()
{
  while (xytheta(2) > M_PI) xytheta(2) -= 2 * M_PI;
  while (xytheta(2) <= -M_PI) xytheta(2) += 2 * M_PI;
}

// Print values of Pose 2D
void CPose2D::print_pose() const
{
  printf(" x =     %f \n y =     %f \n theta = %f \n",xytheta(0),xytheta(1),xytheta(2));
}

// Returns P2 result of displacement D in local frame from actual point (P1.relative_2_absolute)
CPose2D CPose2D::relative_2_absolute(const CPose2D& D) const
{
  Matrix3d R = rotation_matrix(xytheta(2));
  
  CPose2D P2(xytheta + R * D.xytheta);
  P2.angle_correction();
  	
  return P2;
}

// Returns R rotation matrix in 2D (3x3) angle of rotation = theta
Matrix3d CPose2D::rotation_matrix(const double& theta) const
{
  Rotation2Dd rot(theta);
  Matrix3d R = MatrixXd::Identity(3,3);
  R.topLeftCorner(2,2) = rot.toRotationMatrix();
  return R;
}

// Returns jacobian of displacement D in local frame from actual point (P1.relative_2_absolute)
Jacobians CPose2D::relative_2_absolute_jacobian(const CPose2D& D) const
{
  MatrixXd Jp=MatrixXd::Identity(3,3);
  MatrixXd Jd=MatrixXd::Identity(3,3);
  Jacobians J;
  
  double dx=D.xytheta(0);
  double dy=D.xytheta(1);

  double so=sin(xytheta(2));
  double co=cos(xytheta(2));
  
  /* Jacobian of Relative2Absolute w.r.t P
        [1  0 -so*dx-co*dy]
	Jp =  [0  1  co*dx-so*dy]
	      [0  0       1     ] 
  */
  
  Jp(0,2)=-so*dx-co*dy;
  Jp(1,2)=co*dx-so*dy;
  
  /* Jacobian of Relative2Absolute w.r.t D
  Jd = [co -so  0]
       [so  co  0]
       [ 0   0  1] */
  
  Jd(0,0)=co;
  Jd(0,1)=-so;
  Jd(1,0)=so;
  Jd(1,1)=co;
  
  
  J.J1=Jp;
  J.J2=Jd;
  return J;
}

// Returns the relative displacement, D, from the pose P1 (point from calling function) to pose P2 expresed in the frame of P1.
CPose2D CPose2D::absolute_2_relative(const CPose2D& P2) const
{
  Matrix3d R;
  R = rotation_matrix(-xytheta(2));
  CPose2D D(R * (P2.xytheta - xytheta));
  D.angle_correction();
  return D;
}

// Returns the Jacobians of the Absolute2Relative function with respect its parameteres evaluated at P1 (calling function) and P2.
Jacobians CPose2D::absolute_2_relative_jacobian(const CPose2D& P2) const
{

  MatrixXd H1 = MatrixXd::Identity(3,3);
  MatrixXd H2 = MatrixXd::Identity(3,3);
  Jacobians H;
  
  double dx=P2.xytheta(0)-xytheta(0);
  double dy=P2.xytheta(1)-xytheta(1);
  double c1=cos(xytheta(2));
  double s1=sin(xytheta(2));
  
  /*
  H2 = [ c1  s1  0]
      [-s1  c1  0]
      [  0   0  1] */
	   
  H2(0,0)=c1;
  H2(0,1)=s1;
  H2(1,0)=-s1;
  H2(1,1)=c1;
 
  /* 
  H1 =  [-c1 -s1 -s1*dx+c1*dy ]
	[ s1 -c1 -c1*dx-s1*dy ]
	[  0   0    -1        ] */
  
  H1=-1*H2;
  H1(0,2)=-s1*dx+c1*dy;
  H1(1,2)=-c1*dx-s1*dy; 
  
  H.J1=H1;
  H.J2=H2; 
  
  return H;
}
