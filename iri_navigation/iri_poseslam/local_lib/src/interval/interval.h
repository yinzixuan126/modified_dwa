#ifndef _INTERVAL_H
#define _INTERVAL_H
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <stdio.h>
#include <iostream>

using namespace Eigen;

class CInterval
{
  public:
    CInterval();
    CInterval(const CInterval& I);
    CInterval(const MatrixXd& M1, const MatrixXd& M2);
    ~CInterval();
    
    // gets
    MatrixXd get_lower() const;
    MatrixXd get_upper() const;
    uint get_rows() const;
    uint get_cols() const;
    
    // sets
    void set_lower(const MatrixXd& L);
    void set_upper(const MatrixXd& U);
    void set_interval(const MatrixXd& L, const MatrixXd& U);
    
    // interval overloaded operators    
    CInterval& operator= (const CInterval& I);
    
    friend CInterval operator+ (const CInterval& I, const double& d);
    friend CInterval operator+ (const double& d, const CInterval& I);
    friend CInterval operator+ (const CInterval& I, const MatrixXd& M);
    friend CInterval operator+ (const MatrixXd& M, const CInterval& I);
    CInterval operator+ (const CInterval& I) const;
    
    CInterval operator- () const;
    friend CInterval operator- (const CInterval& I, const double& d);
    friend CInterval operator- (const double& d, const CInterval& I);
    friend CInterval operator- (const CInterval& I, const MatrixXd& M);
    friend CInterval operator- (const MatrixXd& M, const CInterval& I);
    CInterval operator- (const CInterval& I) const;
    
    friend CInterval operator* (const CInterval& I, const double& d);
    friend CInterval operator* (const double& d, const CInterval& I);
    friend CInterval operator* (const CInterval& I, const MatrixXd& M);
    friend CInterval operator* (const MatrixXd& M, const CInterval& I);
    CInterval operator* (const CInterval& I) const;
    
    CInterval operator() (const uint& index) const; //only for vector intervals. Only read, for writing use set functions
    CInterval operator() (const uint& row, const uint& col) const; // only read, for writing use set functions

    // interval operations
    CInterval interval_union(const CInterval& I1, const CInterval& I2) const;
    CInterval intersection(const CInterval& I1, const CInterval& I2) const;
    CInterval positive() const;
    MatrixXd diameter() const;
    double diameter(const uint& index) const; //only for vector intervals.
    double diameter(const uint& row, const uint& col) const;
    CInterval pi_2_pi(const CInterval& I) const;
    CInterval multiply_element_wise(const CInterval& I) const;
    CInterval divide_element_wise(const CInterval& I) const; //only for vector intervals.
    CInterval sqrt_element_wise() const;
    
    // utilities
    void print_interval() const;
  private:
    MatrixXd lower;
    MatrixXd upper;
    uint rows;
    uint cols;
};

#endif
