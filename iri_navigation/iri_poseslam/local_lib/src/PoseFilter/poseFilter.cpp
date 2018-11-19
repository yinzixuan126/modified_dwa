#include "poseFilter.h"

using namespace std;

CPoseFilter::CPoseFilter()
{
}

CPoseFilter::CPoseFilter(const CGaussian& gaussian)
:
  initEstimation(gaussian),
  nStates(1),
  nSteps(1),
  dim(initEstimation.get_dim()),
  nLoops(0)
{
  Step2State.reserve(size_t(1000));
  State2Step.reserve(size_t(1000));
  Loops.reserve(size_t(1000));
  
  Step2State.push_back(0);
  State2Step.push_back(0);
}

CPoseFilter::CPoseFilter(const CPoseFilter& pFilter)
:
  initEstimation(pFilter.initEstimation),
  nStates(pFilter.nStates),
  nSteps(pFilter.nSteps),
  Step2State(pFilter.Step2State),
  State2Step(pFilter.State2Step),
  dim(pFilter.dim),
  nLoops(pFilter.nLoops),
  Loops(pFilter.Loops)
{
}

CPoseFilter::~CPoseFilter()
{
}

void CPoseFilter::update(const uint step,const bool overwritePose)
{
  // reserve if needed
  if (State2Step.capacity() == State2Step.size()) State2Step.reserve(State2Step.size() + size_t(1000));
  if (Step2State.capacity() == Step2State.size()) Step2State.reserve(Step2State.size() + size_t(1000));

  // update step-state indexes
  if (overwritePose){
    Step2State.back() = -1;
    State2Step.pop_back();
  }
  else nStates++;
  nSteps++;
  
  State2Step.push_back(step);
  Step2State.push_back(nStates - 1);
}

void CPoseFilter::update_loop(const uint linked)
{
  Loop loop;
  loop.p1 = nSteps - 1;
  loop.p2 = linked;
  nLoops++;
  
  // reserve if needed
  if (Loops.capacity() == Loops.size()) Loops.reserve(Loops.size() + size_t(50));
  
  // add loop into the vector
  Loops.push_back(loop);
}

// GETS
uint CPoseFilter::get_nSteps() const
{
  return nSteps;
}

uint CPoseFilter::get_nStates() const
{
  return nStates;
}

uint CPoseFilter::get_nLoops() const
{
  return nLoops;
}

std::vector<Loop> CPoseFilter::get_loops() const
{
  return Loops;
}

uint CPoseFilter::get_dim() const
{
  return dim;
}

uint CPoseFilter::get_size() const
{
  return dim*nStates;
}

int CPoseFilter::step_2_state(const uint& step) const
{
  return Step2State[step];
}

uint CPoseFilter::state_2_step(const uint& state) const
{
  return State2Step[state];
}
