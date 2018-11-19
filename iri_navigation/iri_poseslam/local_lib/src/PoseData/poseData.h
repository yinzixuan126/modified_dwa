#ifndef _POSEDATA_H
#define _POSEDATA_H
#include "pose.h"
#include "gaussian.h"
#include "interval.h"
#include <eigen3/Eigen/Dense>
#include <iostream>

using namespace Eigen;

class CPoseData
{
  public:
    CPoseData();
    CPoseData(const CPoseData& PD);
    CPoseData(CPose& m,const MatrixXd& S,const MatrixXd& CS,const uint id);
    CPoseData(const VectorXd& m,const MatrixXd& S,const MatrixXd& CS,const uint id);
    CPoseData(const CInterval& iMean, const CInterval& iS, const CInterval& iCS, const std::vector<uint>& iId);
    ~CPoseData();
    
    //GETS
    CPose get_mean() const;
    MatrixXd get_S() const;
    MatrixXd get_CS() const;
    uint get_id() const;
    CGaussian get_gaussian() const;
    CInterval get_interval_mean() const;
    CInterval get_interval_S() const;
    CInterval get_interval_CS() const;
    std::vector<uint> get_interval_id() const;
    bool is_interval() const;
    
    //SETS
    void set_mean(const CPose& m);
    void set_S(const MatrixXd& newS);
    void set_CS(const MatrixXd& newCS);
    void set_id(const uint& newid);
    void set_all(const CPose& m,const MatrixXd& S,const MatrixXd& CS);
    void set_update(const CPose& m,const MatrixXd& S,const MatrixXd& CS);
    void set_all(const VectorXd& m,const MatrixXd& S,const MatrixXd& CS);
    void set_update(const VectorXd& m,const MatrixXd& S,const MatrixXd& CS);
    void set_not_interval();
    
    //METHODS
    CPoseData pd_union(const CPoseData& PD1,const CPoseData& PD2) const;
    void print_pd() const;
  private:
    CPose mean;
    MatrixXd S;
    MatrixXd CS;
    uint id;
    bool isInterval;
    CInterval intervalMean;
    CInterval intervalS;
    CInterval intervalCS;
    std::vector<uint> intervalId;
};

#endif
