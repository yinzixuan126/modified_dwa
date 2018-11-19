#ifndef _POSE2D_H
#define _POSE2D_H
#include <stdio.h>
#include <math.h>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

struct Jacobians
{
  MatrixXd J1;
  MatrixXd J2;
};

class CPose2D
{
  private:
    Vector3d xytheta;
  
  public:
    CPose2D();
    CPose2D(const double& x,const double& y,const double& th);
    CPose2D(const double xyth_new[3]);
    CPose2D(const Vector3d& xyth_new);
    CPose2D(const CPose2D& pose_new);
    ~CPose2D();
    
    //sets
    void set_pose(const double& x,const double& y,const double& th);
    void set_pose(const double xyth_new[3]);
    void set_pose(const Vector3d& xyth_new);
    void set_pose(const CPose2D& pose_new);
    
    //gets
    Vector3d get_vector() const;
    
    //methods
    /**
     * Put the angle into the (-PI, PI] interval
     */
    void angle_correction();
    /**
     * Print values of Pose 2D
     */
    void print_pose() const;
    /**
     * Returns P2 result of displacement D in local frame from actual 
     * point (P1.relative_2_absolute)
     * @param  D [description]
     * @return   [description]
     */
    CPose2D relative_2_absolute(const CPose2D& D) const;
    /**
     * Returns jacobian of displacement D in local frame from actual point (P1.relative_2_absolute)
     * @param  D [description]
     * @return   [description]
     */
    Jacobians relative_2_absolute_jacobian(const CPose2D& D) const;
    /**
     * Returns the relative displacement, D, from the pose P1 
     * (point from calling function) to pose P2 expresed in the frame of P1.
     * @param  P2 [description]
     * @return    [description]
     */
    CPose2D absolute_2_relative(const CPose2D& P2) const;
    /**
     * Returns the Jacobians of the Absolute2Relative function with respect 
     * its parameteres evaluated at P1 (calling function) and P2.
     * @param  P2 [description]
     * @return    [description]
     */
    Jacobians absolute_2_relative_jacobian(const CPose2D& P2) const;
    /**
     * Returns R rotation matrix in 2D (3x3) angle of rotation = theta
     * @param  theta [description]
     * @return       [description]
     */
    Matrix3d rotation_matrix(const double& theta) const;
};

#endif
