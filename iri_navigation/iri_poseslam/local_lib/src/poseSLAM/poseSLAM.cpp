#include "poseSLAM.h"

using namespace Eigen;

CPoseSLAM::CPoseSLAM()
{
}

CPoseSLAM::CPoseSLAM(const MatrixXd& initNoise)
  :
  redundantPose_(false),
  anyClosed_(false)
{
  CGaussian init(VectorXd::Zero(3), initNoise);
  CFlashFilter FF(init);
  FFilter_ = FF;
  CBtree BT(FFilter_.get_PD());
  BTree_ = BT;
}

CPoseSLAM::CPoseSLAM(const MatrixXd& initNoise, const Params& params)
  :
  Parameters_(params),
  redundantPose_(false),
  anyClosed_(false)
{
  CGaussian init(VectorXd::Zero(3), initNoise);
  CFlashFilter FF(init);
  FFilter_ = FF;
  CBtree BT(FFilter_.get_PD());
  BTree_ = BT;
}

CPoseSLAM::CPoseSLAM(const VectorXd& initPose, const MatrixXd& initNoise)
  :
  redundantPose_(false),
  anyClosed_(false)
{
  CGaussian init(initPose, initNoise);
  CFlashFilter FF(init);
  FFilter_ = FF;
  CBtree BT(FFilter_.get_PD());
  BTree_ = BT;
}

CPoseSLAM::CPoseSLAM(const VectorXd& initPose, const MatrixXd& initNoise, const Params& params)
  :
  Parameters_(params),
  redundantPose_(false),
  anyClosed_(false)
{
  CGaussian init(initPose, initNoise);
  CFlashFilter FF(init);
  FFilter_ = FF;
  CBtree BT(FFilter_.get_PD());
  BTree_ = BT;
}

CPoseSLAM::CPoseSLAM(const CGaussian& gaussian)
  :
  FFilter_(gaussian),
  BTree_(FFilter_.get_PD()),
  redundantPose_(false),
  anyClosed_(false)
{
}

CPoseSLAM::CPoseSLAM(const CGaussian& gaussian, const Params& params)
  :
  Parameters_(params),
  FFilter_(gaussian),
  BTree_(FFilter_.get_PD()),
  redundantPose_(false),
  anyClosed_(false)
{
}

CPoseSLAM::CPoseSLAM(const CPoseSLAM& pSLAM)
  :
  Parameters_(pSLAM.Parameters_),
  FFilter_(pSLAM.FFilter_),
  BTree_(pSLAM.BTree_),
  redundantPose_(pSLAM.redundantPose_),
  anyClosed_(pSLAM.anyClosed_)
{
}

CPoseSLAM::~CPoseSLAM()
{
}

// METHODS
void CPoseSLAM::augmentation(const uint& step, const Vector3d& d, const MatrixXd& Q)
{  
  // Insert previous pose in the Binary Tree (if not redundant and not the first)
  if (!redundantPose_ && step > 1) 
    BTree_.insert_tree(FFilter_.get_PD(step - 1));
  
  // state aumentation in FlashFilter
  FFilter_.state_augment(step, CPose(d), Q, redundantPose_, Parameters_.min_cov);
  
  // Present pose isn't redundant for now
  redundantPose_ = false;
  
  // No Loops Closed with present pose
  anyClosed_ = false;
}

void CPoseSLAM::create_candidates_list()
{
  LL_.clear();

  // Search in the binary tree for loop closure candidates
  LL_ = BTree_.tree_detect_loops(FFilter_, Parameters_.matchArea, Parameters_.pdRange);
  
  //printf("%lu loop candidates found in the BTree_ search\n", LL_.size());

  // Compute the information gain of all loop closure candidates
  FFilter_.complete_candidates_list(LL_, Parameters_.LoopNoise);
}

void CPoseSLAM::redundant_evaluation()
{
  if (!anyClosed_ && candidate_.pd > Parameters_.pdRange.second && candidate_.iGain < Parameters_.igRange.first)
    redundantPose_ = true;
  // else if (FFilter_.is_previous_pose(candidate_.step, 1))
  // {
  //     if (anyClosed_)
  //       printf("STATE %i WITH %i - NOT REDUNDANT because Loop was closed\n", FFilter_.get_nSteps() -1, FFilter_.get_PF().state_2_step(candidate_.step));
  //     if (candidate_.pd <= Parameters_.pdRange.second)
  //       printf("STATE %i WITH %i - NOT REDUNDANT because pd_range = %f\n", FFilter_.get_nSteps() -1, FFilter_.get_PF().state_2_step(candidate_.step), candidate_.pd);
  //     if (candidate_.iGain >= Parameters_.igRange.first)
  //       printf("STATE %i WITH %i - NOT REDUNDANT because ig_range = %f\n", FFilter_.get_nSteps() -1, FFilter_.get_PF().state_2_step(candidate_.step), candidate_.iGain);
  // }
}

bool CPoseSLAM::loop_closure_requeriments() const
{
  // printf("LOOP CANDIDATE %i WITH %i ", FFilter_.get_nSteps() -1, candidate_.step);
  // if (redundantPose_)
  //   printf("NO LOOP because REDUNDANT POSE\n");
  // else if (FFilter_.is_previous_pose(candidate_.step, Parameters_.ignorePrevious))
  //   printf("NO LOOP because previous pose\n");
  // else if (!(candidate_.iGain > Parameters_.igRange.second))
  //   printf("NO LOOP because ig_range = %f\n", candidate_.iGain);
  
  return !redundantPose_ && candidate_.iGain > Parameters_.igRange.second && !FFilter_.is_previous_pose(candidate_.step, Parameters_.ignorePrevious) && candidate_.step >= 0;
} 

bool CPoseSLAM::try_loop_closure(MatrixXd& LoopClosureNoise, const VectorXd& LoopClosureD)
{
  if (LoopClosureNoise.determinant() <= 0)
  {
    // printf("POSE SLAM: Wrong loop covariance: det(LoopClosureNoise) <= 0\n");
    return false;
  }
  
  // Outlier test
  if (FFilter_.mahalanobis_distance(candidate_.d, candidate_.S_d, LoopClosureD) > Parameters_.K_mahalanobis)
  {
    // printf("PS: outlier detected! K = %f - Limit = %f  Loop not closed\n", FFilter_.mahalanobis_distance(candidate_.d, candidate_.S_d, LoopClosureD), Parameters_.K_mahalanobis);
    return false;
  }

  // Information threshold
  FFilter_.complete_candidate(candidate_, LoopClosureNoise); // Compute the actual information gain
  if (candidate_.iGain < Parameters_.igRange.second)
  {
    // printf("PS: Not enough informative loop I = %f\n", candidate_.iGain);
    return false;
  }

  // Try to close the loop
  if (FFilter_.state_update(candidate_.step, CPose(LoopClosureD), LoopClosureNoise, Parameters_.min_cov))
  {
    // Update the binary tree
    BTree_.update_tree(FFilter_.get_PF(), FFilter_.get_PD());

    anyClosed_ = true;
    redundantPose_ = false;
    //printf("PS: Informative loop I = %f\n", candidate_.iGain);
    return true;
  }
  else
  {
    // printf("PS: The filter could not update the loop\n");
    return false;
  }
}

void CPoseSLAM::update_candidates_list()
{
  FFilter_.complete_candidates_list(LL_, Parameters_.LoopNoise);
}

// void CPoseSLAM::update_candidates_list(const MatrixXd& Q)
// {
//   FFilter_.complete_candidates_list(LL_, Parameters_.LoopNoise, Q);
// }

bool CPoseSLAM::any_candidate() const
{
  return LL_.size() > 0;
}

void CPoseSLAM::select_best_candidate()
{
  candidate_ = FFilter_.extract_max_information_gain(LL_);
}
    
// SETs
void CPoseSLAM::set_parameters(const Params& params)
{
  Parameters_ = params;
} 

void CPoseSLAM::set_redundant(const bool& redundant)
{
  redundantPose_ = redundant;
}

// GETs
Params CPoseSLAM::get_params() const
{
  return Parameters_;
}

CBtree CPoseSLAM::get_BTree() const
{
  return BTree_;
}

CFlashFilter CPoseSLAM::get_FF() const
{
  return FFilter_;
}

bool CPoseSLAM::is_redundant() const
{
  return redundantPose_;
}

bool CPoseSLAM::is_any_closed() const
{
  return anyClosed_;
}

std::vector<Link> CPoseSLAM::get_LL() const
{
  return LL_;
}

Link CPoseSLAM::get_candidate_link() const
{
  return candidate_;
}

uint CPoseSLAM::get_candidate_step() const
{
  return candidate_.step;
}

std::vector<VectorXd> CPoseSLAM::get_trajectory() const
{
  std::vector<VectorXd> trajectory(get_FF().get_PD().size());
  
  for (uint i = 0; i < trajectory.size(); i++)
    trajectory[i] = get_FF().get_PD()[i].get_mean().get_vector();
  
  return trajectory;
}

std::vector<MatrixXd> CPoseSLAM::get_trajectory_covariance() const
{
  std::vector<MatrixXd> trajectory_cov(get_FF().get_PD().size());
  
  for (uint i = 0; i < trajectory_cov.size(); i++)
    trajectory_cov[i] = get_FF().get_PD()[i].get_S();
  
  return trajectory_cov;
}

std::vector<uint> CPoseSLAM::get_trajectory_steps() const
{
  std::vector<uint> trajectory_steps(get_FF().get_nStates());
  
  for (uint i = 0; i < trajectory_steps.size(); i++)
    trajectory_steps[i] = get_FF().get_PF().state_2_step(i);
  
  return trajectory_steps;
}