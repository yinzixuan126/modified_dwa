#include "interval.h"
 
CInterval::CInterval()
{
}

CInterval::CInterval(const CInterval& I)
  :
  lower(I.lower),
  upper(I.upper),
  rows(I.rows),
  cols(I.cols)
{
}

CInterval::CInterval(const MatrixXd& M1, const MatrixXd& M2)
  :
  lower(M1),
  upper(M2),
  rows(M1.rows()),
  cols(M1.cols())
{
  // check same size
  if (rows!=M2.rows() || cols!=M2.cols()) std::cout << "ERROR: in interval constructor: diferent matrix sizes:" << std::endl << "M1 =" << M1 << std::endl << "M2=" << M2 <<std::endl;
}

CInterval::~CInterval()
{
}

// GETs
MatrixXd CInterval::get_lower() const
{
  return lower;
}

MatrixXd CInterval::get_upper() const
{
  return upper;
}

uint CInterval::get_rows() const
{
  return rows;
}

uint CInterval::get_cols() const
{
  return cols;
}  

// SETs
void CInterval::set_lower(const MatrixXd& L)
{
  lower=L;
}

void CInterval::set_upper(const MatrixXd& U)
{
  upper=U;
}

void CInterval::set_interval(const MatrixXd& L, const MatrixXd& U)
{
  lower=L;
  upper=U;
  rows=L.rows();
  cols=L.cols();
}

// INTERVAL OVERLOADED OPERATORS    

CInterval& CInterval::operator= (const CInterval& I)
{
  lower = I.lower;
  upper = I.upper;
  rows = I.rows;
  cols = I.cols;
  
  return *this;
}

// plus operator
CInterval operator+ (const CInterval& I, const double& d) // CInterval + double
{ 
  MatrixXd Offset = d * MatrixXd::Ones(I.rows, I.cols);
  CInterval result(I.get_lower() + Offset, I.get_upper() + Offset);
  return result;
}

CInterval operator+ (const double& d, const CInterval& I) // double + CInterval
{ 
  return I + d;
}

CInterval operator+ (const CInterval& I, const MatrixXd& M) // CInterval + MatrixXd
{
  CInterval MI(M,M);
  return I + MI;
}

CInterval operator+ (const MatrixXd& M, const CInterval& I) // MatrixXd + CInterval
{
  return I + M;
}

CInterval CInterval::operator+ (const CInterval& I) const // CInterval + CInterval
{
  CInterval result(lower + I.lower, upper + I.upper);
  return result;
}

// minus operator
CInterval CInterval::operator- () const // -CInterval
{
  CInterval result(-upper, -lower);
  return result;
}

CInterval operator- (const CInterval& I, const double& d) // CInterval - double
{
  MatrixXd Offset = d * MatrixXd::Ones(I.get_rows(), I.get_cols());
  CInterval result(I.get_lower() - Offset, I.get_upper() - Offset);
  return result;
}

CInterval operator- (const double& d, const CInterval& I) // double - CInterval
{
  CInterval result(d + (-I));
  return result;
}

CInterval operator- (const CInterval& I, const MatrixXd& M) // CInterval - MatrixXd
{
  CInterval MI(M,M);
  return I - MI;
}

CInterval operator- (const MatrixXd& M, const CInterval& I) // MatrixXd - CInterval
{
  CInterval MI(M,M);
  return M + (-I);
}

CInterval CInterval::operator- (const CInterval& I) const // CInterval - CInterval
{
  CInterval result(lower + (-I));
  return result;
}

// multiplication operator
CInterval operator* (const CInterval& I, const double& d) // CInterval * double
{
  int rows = I.get_rows();
  int cols = I.get_cols();
  MatrixXd L(rows,cols);
  MatrixXd U(rows,cols);
  
  if (d>0)
  {
    L = I.get_lower() * d;
    U = I.get_upper() * d;
  }
  else 
  {
    U = I.get_lower() * d;
    L = I.get_upper() * d;
  }
  
  CInterval result(L, U);
  return result;
}

CInterval operator* (const double& d, const CInterval& I) // double * CInterval
{
  return I * d;
}

CInterval operator* (const CInterval& I, const MatrixXd& M) // CInterval * MatrixXd
{
  uint nr = I.get_rows();
  uint nc = M.cols();
  double s_min, s_max, val1, val2;

  if (I.get_cols() != M.rows()) printf("ERROR: Matrix dimensions in multiplication! \n");

  MatrixXd lower(nr,nc);
  MatrixXd upper(nr,nc);
     
  for (uint i=0; i<nr; i++)
  {
    for (uint j=0; j<nc; j++)
    {
      s_min = 0;
      s_max = 0;

      for (uint k = 0; k < I.get_cols(); k++)
      {
        val1 = I.get_lower()(i,k) * M(k,j);
        val2 = I.get_upper()(i,k) * M(k,j);
        
        s_min = s_min + std::min(val1, val2);
        s_max = s_max + std::max(val1, val2);
      } 
      lower(i,j) = s_min;
      upper(i,j) = s_max;
    }
  }
  CInterval result(lower, upper);
  
  return result;
}

CInterval operator* (const MatrixXd& M, const CInterval& I) // MatrixXd * CInterval
{
  uint nr = M.rows();
  uint nc = I.get_cols();
  double s_min, s_max, val1, val2;

  if (M.cols() != I.get_rows()) printf("ERROR: Matrix dimensions in multiplication! \n");

  MatrixXd lower(nr,nc);
  MatrixXd upper(nr,nc);
    
  for (uint i=0; i<nr; i++)
  {
    for (uint j=0; j<nc; j++)
    {
      s_min = 0;
      s_max = 0;

      for (uint k=0; k < M.cols(); k++)
      {
        val1 = M(i,k) * I.get_lower()(k,j);
        val2 = M(i,k) * I.get_upper()(k,j);
        
        s_min = s_min + std::min(val1, val2);
        s_max = s_max + std::max(val1, val2);
      } 
      lower(i,j) = s_min;
      upper(i,j) = s_max;
    }
  }
  CInterval result(lower, upper);
  
  return result;
}

CInterval CInterval::operator* (const CInterval& I) const // CInterval * CInterval
{
  uint nr = rows;
  uint nc = I.cols;
  double s_min, s_max, val1, val2, val3, val4;

  if (cols != I.rows) printf("ERROR: Matrix dimensions in multiplication! \n");

  MatrixXd L(nr,nc);
  MatrixXd U(nr,nc);
    
  for (uint i=0; i<nr; i++)
  {
    for (uint j=0; j<nc; j++)
    {
      s_min = 0;
      s_max = 0;

      for (uint k=0; k<cols; k++)
      {
        val1 = lower(i,k) * I.lower(k,j);
        val2 = lower(i,k) * I.upper(k,j);
        val3 = upper(i,k) * I.lower(k,j);
        val4 = upper(i,k) * I.upper(k,j);
        
        s_min = s_min + std::min(std::min(val1, val2), std::min(val3, val4));
        s_max = s_max + std::max(std::max(val1, val2), std::max(val3, val4));
      } 
      L(i,j) = s_min;
      U(i,j) = s_max;
    }
  }
  CInterval result(L,U);
  return result;
}

CInterval CInterval::operator() (const uint& index) const // CInterval(int)
{
  //only for vector intervals. Only read, for writing use set functions
  MatrixXd low(1,1);
  low(0,0) = 0;
  MatrixXd up(1,1);
  up(0,0) = 0;

  if (rows == 1)
  {
    low(0,0) = lower(0, index);
    up(0,0) = upper(0, index);
  }
  else if (cols == 1)
  {
    low(0,0) = lower(index, 0);
    up(0,0) = upper(index, 0);
  }
  else printf("ERROR: operator(int) in Interval with more than one dimension\n");
  
  CInterval result(low,up);
  
  return result;
}

CInterval CInterval::operator() (const uint& row, const uint& col) const // CInterval(int, int)
{
  // only read, for writing use set functions
  MatrixXd low(1,1);
  low(0,0) = lower(row, col);
  MatrixXd up(1,1);
  up(0,0) = upper(row, col);
   
  CInterval result(low, up);
  
  return result;
}


// INTERVAL OPERATIONS
CInterval CInterval::interval_union(const CInterval& I1, const CInterval& I2) const
{
  // Hull of two interval matrices.
  // Produces the hull of two interval matrices I1, I2, i.e., the interval matrix that includes the two inputs.
  
  if (I1.rows!=I2.rows || I1.cols!=I2.cols) std::cout << "ERROR: dimension dismatch in interval union!" << std::endl;
  
  CInterval result(I1.lower.cwiseMin(I2.lower), I1.upper.cwiseMax(I2.upper));
  
  return result;
}

CInterval CInterval::intersection(const CInterval& I1, const CInterval& I2) const
{
  // Intersection of two interval matrices, I1, I2.
  // If one of the intervals is empty, an error is triggered.
  
  if (I1.rows!=I2.rows || I1.cols!=I2.cols) std::cout << "ERROR: dimension dismatch in interval intersection!" << std::endl;
  CInterval result(I1.lower.cwiseMax(I2.lower), I1.upper.cwiseMin(I2.upper));
  
  return result;
}

CInterval CInterval::positive() const
{
  // Positive interval
  // It returns an interval with the positive part of interval input.
  CInterval result(lower.cwiseMax(MatrixXd::Zero(rows, cols)), upper.cwiseMax(MatrixXd::Zero(rows,cols)));
  
  return result;
}

MatrixXd CInterval::diameter() const
{
  // Diameter of an interval matrix, I. 
  // It returns a matrix, with the size of the interval at the corresponding position of the input matrix.

  return upper - lower;
}

double CInterval::diameter(const uint& index) const
{
  // Diameter of an interval matrix element, I(index) for vector intervals only. 
  // It returns a double, with the size of the interval at the corresponding position of the input matrix.
  double d = 0;
  
  if (rows == 1)
  {
    d = upper(0, index) - lower(0, index);
  }
  else if (cols == 1)
  {
    d = upper(index, 0) - lower(index, 0);
  }
  else printf("ERROR: diameter(int) with more than one dimension\n");
  
  return d;
}

double CInterval::diameter(const uint& row, const uint& col) const
{
  // Diameter of an interval matrix element, I(row,col). 
  // It returns a double, with the size of the interval at the corresponding position of the input matrix.
  
  return upper(row, col) - lower(row, col);
}

CInterval CInterval::pi_2_pi(const CInterval& I) const
{
  // Forces an angular interval to be in [-pi,pi].
  // Adjusts an angular interval so that it is included in [-pi,pi].
  // We shift the interval by +/-2 pi keeping the result that maximizes the intersection with [-pi,pi].

  double dp = 2 * M_PI;
  MatrixXd pi(1,1);
  pi(0,0)=M_PI;
  
  CInterval pp(-pi, pi);
  CInterval Icn = intersection(I, pp);
  CInterval Iup = intersection(I + dp, pp);
  CInterval Ilo = intersection(I - dp, pp);
  CInterval result(I);
  
  if (Iup.diameter(0) > Icn.diameter(0))
    result = Iup;
  
  else if (Ilo.diameter(0) > Icn.diameter(0))
    result = Ilo;
  
  return result;
}

CInterval CInterval::multiply_element_wise(const CInterval& I) const
{
  MatrixXd L = MatrixXd::Zero(rows,cols);
  MatrixXd U = MatrixXd::Zero(rows,cols);
  double val1, val2, val3, val4;
  
  if (rows != I.rows || cols != I.cols) printf("ERROR: Matrix dimensions in multiply_element_wise! \n");
  else
  {
    for (uint i=0; i < rows; i++)
    {
      for (uint j=0; j < cols; j++)
      {
        val1 = lower(i,j) * I.lower(i,j);
        val2 = lower(i,j) * I.upper(i,j);
        val3 = upper(i,j) * I.lower(i,j);
        val4 = upper(i,j) * I.upper(i,j);
        
        L(i,j) = std::min(std::min(val1, val2), std::min(val3, val4));
        U(i,j) = std::max(std::max(val1, val2), std::max(val3, val4));
      }
    }
  }  
  CInterval result(L,U);
  return result;
}

CInterval CInterval::divide_element_wise(const CInterval& I) const
{
  MatrixXd L = MatrixXd::Zero(rows,cols);
  MatrixXd U = MatrixXd::Zero(rows,cols);
  double val1, val2, val3, val4;
  
  if (rows != I.rows || cols != I.cols) printf("ERROR: Matrix dimensions in divide_element_wise! \n");
  else
  {
    for (uint i=0; i < rows; i++)
    {
      for (uint j=0; j < cols; j++)
      {
        val1 = lower(i,j) / I.lower(i,j);
        val2 = lower(i,j) / I.upper(i,j);
        val3 = upper(i,j) / I.lower(i,j);
        val4 = upper(i,j) / I.upper(i,j);
        
        L(i,j) = std::min(std::min(val1, val2), std::min(val3, val4));
        U(i,j) = std::max(std::max(val1, val2), std::max(val3, val4));
      }
    }
  }  
  CInterval result(L,U);
  return result;
}

CInterval CInterval::sqrt_element_wise() const
{
  MatrixXd L = MatrixXd::Zero(rows,cols);
  MatrixXd U = MatrixXd::Zero(rows,cols);
  double val1, val2;
  
  for (uint i=0; i < rows; i++)
  {
    for (uint j=0; j < cols; j++)
    {
      val1 = sqrt(lower(i,j));
      val2 = sqrt(upper(i,j));
      
      L(i,j) = std::min(val1, val2);
      U(i,j) = std::max(val1, val2);
    }
  }
  CInterval result(L, U);
  return result;
}

// UTILITIES
void CInterval::print_interval() const
{
  std::cout << "lower:" << std::endl << lower << std::endl;
  std::cout << "upper:" << std::endl << upper << std::endl;
}
