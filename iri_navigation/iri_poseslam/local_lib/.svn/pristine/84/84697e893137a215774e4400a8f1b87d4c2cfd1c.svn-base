#ifndef _GAUSSIAN_H
#define _GAUSSIAN_H
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <stdio.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Sparse>
#include <math.h>
#include <iostream>

using namespace Eigen;

class CGaussian
{
  public:
    CGaussian();
    CGaussian(const CGaussian& gaussian);
    CGaussian(const Eigen::VectorXd& v,const Eigen::MatrixXd& s);    
    ~CGaussian();
    
    // gets
    VectorXd get_m() const;
    uint get_dim() const;
    MatrixXd get_S() const;
    SparseMatrix<double> get_iS() const;
    double get_d() const;
    double get_ct() const;
    
    // functions
    double gaussian_CPD(double t) const;
    
  private:
    VectorXd m;
    uint dim;
    MatrixXd S;
    SparseMatrix<double> iS;
    double d;
    double ct;
};

#endif
