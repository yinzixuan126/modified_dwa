#define THRES_MIN 1e-10
#include "gaussian.h"

// Default constructor 
CGaussian::CGaussian()
{
}

// Constructor copying another Gaussian 
CGaussian::CGaussian(const CGaussian& gaussian)
:
  m(gaussian.m),
  dim(gaussian.dim),
  S(gaussian.S),
  iS(gaussian.iS),
  d(gaussian.d),
  ct(gaussian.ct)
{
}

// Constructor from mean and covariance matrix
CGaussian::CGaussian(const VectorXd& v,const MatrixXd& s)
:
  m(v),
  dim(v.rows()),
  S(s)
{
  
  if (S.maxCoeff()<THRES_MIN)
  { 
    S = MatrixXd::Zero(dim, dim);
    d = 0;
    iS = (1e3 * MatrixXd::Identity(dim, dim)).sparseView(); //it should be infinite
    ct = 0;
  }
  else
  {
    //inverse and Cholesky decomposition
    LLT<MatrixXd> lltOfS(S); // compute the Cholesky decomposition of S
    MatrixXd L = lltOfS.matrixL(); // retrieve factor L in the decomposition
    MatrixXd iL = L.inverse();
    iS = (iL * iL.transpose()).sparseView();
    
    d = pow(L.diagonal().prod(), 2);
    ct = 1 / sqrt(pow((2 * M_PI), dim) * d);
  }
}

//IDEA: CGaussian::CGaussian(const CPose& v,const CCovariance& s)

CGaussian::~CGaussian()
{
}

// GETS
VectorXd CGaussian::get_m() const
{
  return m;
}

uint CGaussian::get_dim() const
{
  return dim;
}

MatrixXd CGaussian::get_S() const
{
  return S;
}

SparseMatrix<double> CGaussian::get_iS() const
{
  return iS;
}

double CGaussian::get_d() const
{
  return d;
}

double CGaussian::get_ct() const
{
  return ct;
}

// FUNCTIONS

// Computes the Gaussian cummulative probability distribution for a point 't' in a uni-dimensional Gaussian.
double CGaussian::gaussian_CPD(double t) const
{
  if (dim > 1) printf("ERROR: Gaussian CPD is only defined one one-dimensional Gaussians\n");
  
  return 0.5 * (1 + erf((t - m(0)) / sqrt(2 * S(0,0))));
}

