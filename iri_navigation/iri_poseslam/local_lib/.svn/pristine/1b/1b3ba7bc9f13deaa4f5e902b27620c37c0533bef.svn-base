#ifndef _POSEFILTER_H
#define _POSEFILTER_H
#include "gaussian.h"
#include "pose.h"
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

struct Loop
{
  uint p1;
  uint p2;
  //CPose displacement;
  //Eigen::MatrixXf noise;
};

class CPoseFilter
{
  public:
    CPoseFilter();
    CPoseFilter(const CGaussian& gaussian);
    CPoseFilter(const CPoseFilter& pFilter);
    ~CPoseFilter();
    
    void update(const uint step,const bool overwritePose);
    void update_loop(const uint linked);
    
    //gets
    uint get_nSteps() const;
    uint get_nStates() const;
    uint get_nLoops() const;
    std::vector<Loop> get_loops() const;
    uint get_dim() const;
    uint get_size() const;
    int step_2_state(const uint& step) const;
    uint state_2_step(const uint& state) const;
    
  private:
    CGaussian initEstimation;
    uint nStates;
    uint nSteps;
    std::vector<int> Step2State;
    std::vector<uint> State2Step;
    uint dim;
    uint nLoops;
    std::vector<Loop> Loops;
};

#endif
