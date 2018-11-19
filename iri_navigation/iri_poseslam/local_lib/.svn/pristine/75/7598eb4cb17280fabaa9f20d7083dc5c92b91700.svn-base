#ifndef _POSE_H
#define _POSE_H
#include "pose2D.h"

class CPose
{
  private:
    CPose2D Pose2D;
    //CPose3D Pose3D;
    bool is2D;
    
  public:
    CPose(const double& x=0,const double& y=0,const double& th=0);
    CPose(const double xyth_new[3]);
    CPose(const Vector3d& xyth_new);
    CPose(const CPose2D& pose_new);
    ~CPose();
    
    //SETS 2D
    void set_pose(const double& x,const double& y,const double& th);
    void set_pose(const double xyth_new[3]);
    void set_pose(const Vector3d& xyth_new);
    void set_pose(const CPose2D& pose_new);
    void set_pose(const CPose& pose_new);
    
    //GETS
    VectorXd get_vector() const;
    
    //METHODS
    void print_pose() const;
    CPose relative_2_absolute(const CPose& D) const;
    Jacobians relative_2_absolute_jacobian(const CPose& D) const;
    CPose absolute_2_relative(const CPose& P2) const;
    Jacobians absolute_2_relative_jacobian(const CPose& P2) const;
};

#endif
