#include "flashFilter.h"

CFlashFilter::CFlashFilter()
{
}

CFlashFilter::CFlashFilter(const CFlashFilter& FF)
  :
  lambda_(FF.lambda_),
  secondLastMarginalLambda_(FF.secondLastMarginalLambda_),
  mu_(FF.mu_),
  eta_(FF.eta_),
  secondLastEta_(FF.secondLastEta_),
  PD_(FF.PD_),
  PF_(FF.PF_),
  F_(FF.F_),
  secondLastF_(FF.secondLastF_),
  lastQ_(FF.lastQ_)
{
}

CFlashFilter::CFlashFilter(const CGaussian& gaussian)
  :
  PF_(gaussian)
{
  uint dim = gaussian.get_dim();
  if (gaussian.get_d() < 1e-9)
    lambda_ = (1e3 * (MatrixXd::Identity(dim, dim))).sparseView(); //should be infinite, we put 1000*Identity
  else
    lambda_ = gaussian.get_iS();
  
  secondLastMarginalLambda_ = lambda_;
  mu_ = gaussian.get_m();
  eta_ = lambda_ * mu_;
  secondLastEta_ = VectorXd::Zero(dim);
  CPoseData pData(mu_, (sparse_2_dense(lambda_)).inverse(), (sparse_2_dense(lambda_)).inverse(), 0);
  PD_.reserve(100);
  PD_.push_back(pData);
  F_ = MatrixXd::Identity(dim, dim);
  secondLastF_ = F_;
  lastQ_ = MatrixXd::Zero(dim, dim);
  // t_simplicial_= 0;
  // t_supernodal_ = 0;
}

CFlashFilter::~CFlashFilter()
{
}

// METHODS

void CFlashFilter::state_augment(const uint& step, const CPose& d, const MatrixXd& Qd, const bool& overwritePose, const double& min_cov)
{
  //printf("New state - step %i:\n", step);
  //cout << sparse_2_dense(lambda_) << endl;
  //cout << "Qd = "<< endl << Qd << endl;

  // reserve if needed
  if (PD_.size() == PD_.capacity())
    PD_.reserve(PD_.size() + 100);

  uint dim = PF_.get_dim(); //dimensionality of the pose space
  CPose prevP;
  CPose newP;
  MatrixXd Fn(dim, dim);
  MatrixXd Su = Qd;
  make_symmetric(Su);
  MatrixXd Q(dim, dim);

  // Zero or negative entries in the diagonal of the link covariance
  if ((Su.diagonal().array() < min_cov).any())
  {
    //cout << "PS: Too much small entry in odometry covariance diagonal"<< endl << Su << endl;
    Su.diagonal() = Su.diagonal().array().max(min_cov).matrix();
    //cout << "PS: Too much small entry in odometry covariance diagonal. Fixed:"<< endl << Su << endl;
  }
  if (!is_semi_pos_def(Su))
  {
    //cout << "PS: Not positive definite odometry covariance" << endl << Su << endl;
    Su = Su.array().max((MatrixXd::Identity(dim,dim) * min_cov).array()).matrix();
    //cout << "PS: Not positive definite odometry covariance. Fixed:"<< endl << Su << endl;
  }
  if (!((Su - Su).array() == (Su - Su).array()).all())
  {
    //cout << "ERROR: Not finite entry in odometry covariance" << endl << Su << endl;
    Su = MatrixXd::Identity(dim,dim) * max(min_cov, Su.maxCoeff());
    //cout << "PS: Not finite entry in odometry covariance. Fixed:"<< endl << Su << endl;
  } 

  // OVERWRITE CASE (previous pose is redundant)
  if (overwritePose)
  {
    uint idxPrev = (PF_.get_nStates() - 2) * dim; //index where begin the dim-components of Last non redundant state

    // Poses
    CPose redundantP(mu_.tail(dim)); // Pose that will be removed
    prevP.set_pose(mu_.segment(idxPrev, dim)); // Last know pose that we will keep
    newP.set_pose(redundantP.relative_2_absolute(d)); // mean for the new pose
    // Jacobians used to linarly propagate the error from previous redundant pose to the new one
    Jacobians J23 = redundantP.relative_2_absolute_jacobian(d);
    // Noise propagation from redundant pose
    Q = J23.J1 * lastQ_ * J23.J1.transpose() + J23.J2 * Su * J23.J2.transpose();
    // Jacobian w.r.t. the previous non-redundant pose (prevP)
    Fn = prevP.relative_2_absolute_jacobian(prevP.absolute_2_relative(newP)).J1;

    // UNDO THE LAST STATE AUGMENT
    // UNaugment lambda
    erase_sparse_block(lambda_, idxPrev, 2 * dim, idxPrev, 2 * dim);
    insert_sparse_block(secondLastMarginalLambda_, lambda_, idxPrev, idxPrev);
    // UNaugment eta
    VectorXd etaN = eta_.tail(dim);
    eta_.segment(idxPrev, dim) = secondLastEta_;
    // UNaugment F
    F_ = secondLastF_;
    // Delete the last PoseData
    PD_.pop_back();
  }

  // NOT OVERWRITE CASE (previous pose isn't redundant)
  else
  {
    // Poses
    prevP.set_pose(mu_.tail(dim)); // mean for the previous pose (used as a linealization point)
    newP.set_pose(prevP.relative_2_absolute(d)); // mean for the new pose
    // Jacobians used to linarly propagate the error from previous pose to the new one
    Jacobians J = prevP.relative_2_absolute_jacobian(d);
    Fn = J.J1;
    // displacement noise
    Q = J.J2 * Su * J.J2.transpose(); 

    // STORE LAST POSE ATTRIBUTES (used to undo the state augment when overwriting)
    // Store marginal lambda
    secondLastMarginalLambda_ = lambda_.bottomRightCorner(dim, dim);
    // Store accumulated jacobian F
    secondLastF_ = F_;
    // Store eta pose
    secondLastEta_ = eta_.tail(dim);

    // RESIZE
    // Resize eta
    eta_.conservativeResize(eta_.size() + dim);
    // Resize mu
    mu_.conservativeResize(mu_.size() + dim); 
    // Resize lambda
    lambda_.conservativeResize(lambda_.rows() + dim, lambda_.cols() + dim);
  }

  // Inverse of odometry covariance
  MatrixXd iQ;
  make_symmetric(Q);    
  if (Q.maxCoeff() < min_cov)
  {
    //cout << "maxCoeff too small..." << endl << iQ << endl;
    iQ = 1e3 * MatrixXd::Identity(dim, dim); //it should be infinite
  }
  else
  {
    //if (Q.determinant() == 0) cout << "ERROR: Q not invertible!" << endl << Q << endl;  
    //if (!((Q - Q).array() == (Q - Q).array()).all()) cout << "ERROR: Q contains not finite numbers!" << endl << Q << endl;  
    iQ = Q.inverse();
    make_symmetric(iQ);
    //if (!((iQ - iQ).array() == (iQ - iQ).array()).all()) cout << "ERROR: iQ contains not finite numbers!" << endl << iQ << endl;  
  }
  
  // AUGMENTATION
  // eta
  VectorXd etaN = iQ * (newP.get_vector() - Fn * prevP.get_vector());
  eta_.segment(eta_.size() - 2 * dim, dim) -= Fn.transpose() * etaN; //last state
  eta_.tail(dim) = etaN; //new state

  // mu
  mu_.tail(dim) = newP.get_vector();

  // lambda
  MatrixXd updateLambda(2 * dim, 2 * dim);
  updateLambda.block(0, 0, dim, dim) = Fn.transpose() * iQ * Fn;
  updateLambda.block(0, dim, dim, dim) = -Fn.transpose() * iQ;
  updateLambda.block(dim, 0, dim, dim) = -iQ * Fn;
  updateLambda.block(dim, dim, dim, dim) = iQ;
  make_symmetric(updateLambda);
  // if (!((updateLambda - updateLambda).array() == (updateLambda - updateLambda).array()).all())
  //   cout << "ERROR: updateLambda contains not finite numbers!" << endl << updateLambda << endl;
  add_sparse_block(updateLambda, lambda_, lambda_.rows() - 2 * dim, lambda_.cols() - 2 * dim);
  //cout << "last Marginal Lambda" << endl << sparse_2_dense(secondLastMarginalLambda_) << endl;

  // Pose Data
  MatrixXd prevS = PD_.back().get_S(); // Previous pose marginal covariance 
  PD_.back().set_CS(prevS * F_.inverse()); // Update last pose cross covariance
  MatrixXd newS = Fn * prevS * Fn.transpose() + Q; // Store new pose data
  // if (!((newS - newS).array() == (newS - newS).array()).all() || (newS.diagonal().array() <= 0).any() || !is_semi_pos_def(newS)) 
  //   cout << "ERROR: newS contains not finite numbers!" << endl << newS << endl << "prevS" << endl << prevS << endl << "Q" << endl << Q << endl << "Fn" << endl << Fn << endl;
  CPoseData newPD(newP, newS, MatrixXd::Identity(dim,dim), step); // Cross covariance factor will be updated the next augmentation
  PD_.push_back(newPD);
  
  // Generic Pose Filter (steps, states, etc.)
  PF_.update(step, overwritePose);  

  // Accumulated jacobian
  F_ = F_ * Fn.transpose();

  // Store odometry noise and jacobian (used when overwriting)
  lastQ_ = Q;

  //printf("State augmented:\n det(Lambda) = %f\n Lambda = ", sparse_2_dense(lambda_).determinant());
  //cout << sparse_2_dense(lambda_) << endl;
  if (sparse_2_dense(lambda_).determinant() <= 0)
  {
    printf("ERROR: After an augmentation, det(Lambda) <= 0! %f\n", sparse_2_dense(lambda_).determinant());
    cout << "Q" << endl << Q << endl << "iQ" << endl << iQ << endl << "Fn" << endl << Fn << endl;
    cout << "updateLambda" << endl << updateLambda << endl << "lambda updated" << endl << lambda_.bottomRightCorner(2*dim, 2* dim) << endl;
    cout << "d" << endl << d.get_vector() << endl << "Qd" << Qd << endl << "Su" << Su << endl;
  }
}

bool CFlashFilter::state_update(const uint& step, const CPose& d, MatrixXd& Sy, const double& min_cov)
{
  //  step: The step to be linked with the current robot's pose to form a loop.
  //  d: Displacement between the current robot's pose and pose 'i' according to the sensors readings.
  //  Sy: Noise of the sensor
  
  //printf("TRYING LOOP states %i-%i... \n", PF_.step_2_state(step), PF_.get_nStates()-1);
  
  uint state = PF_.step_2_state(step);
  uint currentState = PF_.get_nStates() - 1;
  uint dim = PF_.get_dim(); //dimensionality of the pose space
  uint idxI = dim * state;
  uint idxN = dim * currentState;
  uint ne = PF_.get_size();

  if (Sy.determinant() <= 0)
  {
    //cout << "ERROR: det(Sy) <= 0" << endl << Sy << endl;
    return false;
  }
  if ((Sy.diagonal().array() < min_cov).any())
  {
    //cout << "PS: Too much small entry in odometry covariance diagonal"<< endl << Sy << endl;
    Sy.diagonal() = Sy.diagonal().array().max(min_cov).matrix();
    //cout << "PS: Too much small entry in loop covariance diagonal. Fixed:"<< endl << Sy << endl;
  }
  if (PF_.step_2_state(step) == -1) 
  {
    //printf("ERROR: Trying to close a loop with a removed pose\n");
    return false;
  }
  if (state == currentState)
  {
    //printf("ERROR: Self loop in update flash loop\n");
    return false;
  }

  CPose Pi(mu_.segment(idxI,dim));
  CPose Pn(mu_.tail(dim));
  
  // Compute Jacobians
  Jacobians H12 = Pi.absolute_2_relative_jacobian(Pn); //Hi = H12.J1 and Hn = H12.J2;
  MatrixXd Hni = concatenate_matrices(H12.J2, H12.J1, true);
  
  // Compute the innovation and the inverse of its Cholesky factor.
  MatrixXd M = get_marginal_covariance(PF_.state_2_step(state));
  
  if ( (M.diagonal().array() <= min_cov).any())
  {
    //cout << "ERROR: M has 0 or negative covariance!" << endl;// << M << endl;
    return false;
  }
  if (!((M - M).array() == (M - M).array()).all())
  {
    //cout << "ERROR: M has not finite entries!" << endl;// << M << endl;
    return false;
  }
  if (!is_semi_pos_def(M))
  {
    //cout << "ERROR: M is not pos.def!" << endl << M.eigenvalues() << endl;
    return false;
  }
  MatrixXd S = Hni * M * Hni.transpose() + Sy;
  make_symmetric(S);
  if (!is_semi_pos_def(S))
  {
    //cout << "ERROR: S is not pos.def!" << endl;// << S << endl;
    make_semi_pos_def(S);
  }
  //if ( (S.diagonal().array() <= min_cov).any() || !((S - S).array() == (S - S).array()).all()) cout << "ERROR: S has too much small covariance!" << endl << S << endl;
  //if (!is_semi_pos_def(S)) cout << "ERROR: S is not pos.def!" << endl << S << endl;

  MatrixXd iV = S.llt().matrixL(); // compute the Cholesky decomposition of S
  MatrixXd V = iV.inverse();
  //if (!((V - V).array() == (V - V).array()).all()) cout << "ERROR: V has not finite entries!" << endl << V << endl;
  
  // Compute the i-th block column of the current covariance matrix
  MatrixXd Ii = MatrixXd::Zero(ne,dim);
  Ii.block(idxI, 0, dim, dim) = MatrixXd::Identity(dim,dim);

  // COMPARING CHOLESKY DECOMPOSITION METHODS
  bool succeed = false;
  // Simplicial LLT
  MatrixXd Si;
  // t0_.set();
  SimplicialLLT<SparseMatrix<double> > solver;
  solver.compute(lambda_);
  if (solver.info() != Success) 
    printf("Simplicial Cholesky decomposition failed!!\n det(Lambda) = %f\n", sparse_2_dense(lambda_).determinant());
  else
  {
    succeed = true;
    Si = solver.solve(Ii);
  }
  // t1_.set();
  // t_simplicial_ += 1e-9 * (t1_ - t0_).nanoseconds();
  
  // Supernodal
  // MatrixXd Si2;
  // t0_.set();
  // CholmodSupernodalLLT<SparseMatrix<double> > solver2;
  // solver2.compute(lambda_);
  // if (solver2.info() != Success) 
  //   printf("Supernodal Cholesky decomposition failed!!\n det(Lambda) = %f\n", sparse_2_dense(lambda_).determinant());
  // else
  // {
  //   succeed = true;
  //   Si2 = solver2.solve(Ii);
  // }
  // t1_.set();
  // t_supernodal_ += 1e-9 * (t1_ - t0_).nanoseconds();

  if (!succeed)
    return false;

  // Compute the n-th block column of the current covariance matrix
  MatrixXd Sn = MatrixXd::Zero(ne,dim);
  for (uint i = 0; i < currentState; i++)
    Sn.block(i * dim, 0, dim, dim) = PD_.at(i).get_CS();
  Sn = Sn * F_;
  Sn.block(idxN, 0, dim, dim) = M.block(0, 0, dim, dim);
  
  // Concatenation Sni=[Sn Si]
  MatrixXd Sni = concatenate_matrices(Sn, Si, true);
  
  // Compute the block column used to update the covariance and the mean
  MatrixXd B = Sni * Hni.transpose() * V.transpose();
  
  // New mu
  VectorXd newMu = mu_ + B * V * (d.get_vector() - Pi.absolute_2_relative(Pn).get_vector());
  angle_correction_mu(newMu); // all angles into the [-PI, PI] interval
  
  // Check if the loop closure id a "good" update
  if (!((newMu - newMu).array() == (newMu - newMu).array()).all())
  {
    //printf("ERROR: newMu has not finite entries!\n");
    return false;
  }
  if (!((B - B).array() == (B - B).array()).all())
  {
    //printf("ERROR: B has not finite entries!\n");
    return false;
  }
  if (!((Sn - Sn).array() == (Sn - Sn).array()).all())
  {
    //printf("ERROR: Sn has not finite entries!\n");
    return false;
  }
  // if ( mahalanobis_distance(mu_, get_pose_covariance(PF_.get_nSteps() - 1), newMu.tail(3)) > 50 )
  // {
  //   printf("ERROR: newMu mahalanobis distance is higher than 10!\n");
  //   return false;
  // }

  // Compute the last column of the new covariance matrix
  MatrixXd Bn = B.block(idxN, 0, dim, B.cols());
  MatrixXd newSn = Sn - B * Bn.transpose();
  if (!((newSn - newSn).array() == (newSn - newSn).array()).all())
  {
    //printf("ERROR: newSn has not finite entries!\n");
    return false;
  }

  // Compute the new marginal covariances
  vector<CPoseData> newPD = PD_;
  for (uint i = 0; i < currentState; i++)
  {
    MatrixXd Bi = B.block(dim * i, 0, dim, B.cols());
    MatrixXd dSii = -Bi * Bi.transpose();
    make_symmetric(dSii);
    newPD.at(i).set_update(newMu.segment(dim * i, dim), dSii, newSn.block(dim * i, 0, dim, newSn.cols()));

    MatrixXd newSii = newPD.at(i).get_S();
    if ((newSii.diagonal().array() < min_cov).any())
    {
      //cout << "ERROR: New marginal " << i << "th block has too much small covariance!" << endl << newSii << endl;
      //newSii.diagonal() = newSii.diagonal().array().max(min_cov).matrix();
      newSii.diagonal() = newSii.diagonal().array().abs().max(min_cov).matrix();
      //cout << "Fixed marginal " << endl << newSii << endl;
      //return false;
    }
    if (!((newSii - newSii).array() == (newSii - newSii).array()).all())
    {
      // cout << "ERROR: New marginal " << i << "th block has non finite covariance!" << endl;// << newSii << endl;
      return false;
    }
    if (!is_semi_pos_def(newSii))
    {
      // cout << "ERROR: New Marginal " << i << "th block is not pos.def!" << endl;
      return false;
      //make_semi_pos_def(newPD.at(i).get_S());
    }
  }
  MatrixXd dSnn = -Bn * Bn.transpose();
  make_symmetric(dSnn);
  newPD.at(currentState).set_update(newMu.tail(dim), dSnn, MatrixXd::Identity(dim, dim));
  newPD.at(currentState).set_CS(newPD.at(currentState).get_S());
  
  MatrixXd newSnn = newPD.at(currentState).get_S();
  if ((newSnn.diagonal().array() < min_cov).any())
  {
    // cout << "ERROR: Last new marginal block has too much small covariance!" << endl << newSnn << endl;
    newSnn.diagonal() = newSnn.diagonal().array().max(min_cov).matrix();
    // cout << "Fixed marginal " << endl << newSnn << endl;
    //return false;
  }
  if (!((newSnn - newSnn).array() == (newSnn - newSnn).array()).all())
  {
    // cout << "ERROR: Last new marginal has non finite covariance!" << endl;// << newSnn << endl;
    return false;
  }
  if (!is_semi_pos_def(newSnn))
  {
    // cout << "ERROR: Last new Marginal is not pos.def!" << endl;
    return false;
    //make_semi_pos_def(newPD.at(i).get_S());
  }
  
  // UPDATES
  // PoseData
  PD_ = newPD;
  // mu
  mu_ = newMu;
  // lambda
  MatrixXd iSy = Sy.inverse();
  make_symmetric(iSy);
  MatrixXd Hi = H12.J1;
  MatrixXd Hn = H12.J2;
  add_sparse_block(Hi.transpose() * iSy * Hi, lambda_, idxI, idxI);
  add_sparse_block(Hi.transpose() * iSy * Hn, lambda_, idxI, idxN);
  add_sparse_block(Hn.transpose() * iSy * Hi, lambda_, idxN, idxI);
  add_sparse_block(Hn.transpose() * iSy * Hn, lambda_, idxN, idxN);
  if (sparse_2_dense(lambda_).determinant() <= 0) 
    printf("ERROR: det(Lambda) <= 0! %f\n", sparse_2_dense(lambda_).determinant());
  // eta
  eta_ = lambda_ * mu_;
  // accumulated jacobian
  F_ = MatrixXd::Identity(dim, dim);
  // Pose Filter
  PF_.update_loop(step);

  //printf("CLOSED!\n");
  // if (PF_.get_nLoops() % 20 == 0) 
  //   printf("%u loops closed\n\tSupernodal average time = %f\n\tSimplicial average time = %f\n",PF_.get_nLoops(), t_supernodal_ / PF_.get_nLoops(), t_simplicial_ / PF_.get_nLoops());

  return true;
}

// Information gain of stablishing a link between the current robot's and that at time 'step' with a sensor with noise R.
void CFlashFilter::complete_candidate(Link& link, const MatrixXd& R) const
{  
  MatrixXd MarginalSigma = get_marginal_covariance(link.step);
  
  CPose PoseN(get_pose_mean(get_nSteps() - 1));
  CPose PoseI(get_pose_mean(link.step));
  
  Jacobians H = PoseI.absolute_2_relative_jacobian(PoseN);
  MatrixXd Hni = concatenate_matrices(H.J2, H.J1, true);
  MatrixXd S_d = Hni * MarginalSigma * Hni.transpose();
  MatrixXd S = R + S_d;
  
  link.iGain = 0.5 * log(S.determinant() / R.determinant());
  link.d = PoseI.absolute_2_relative(PoseN).get_vector();
  link.S_d = S_d;

  // if (isnan(link.iGain))
  //   cout << "nan information gain candidate:" << endl
  //   << "  step " << link.step << endl
  //   << "  det(Sd) = " << link.S_d.determinant() << endl
  //   << "  Sd" << endl << link.S_d << endl
  //   << "  det(S) = " << S.determinant() << endl
  //   << "  S" << endl << S << endl
  //   << "  det(MarginalSigma) = " << MarginalSigma.determinant() << endl
  //   << "  MarginalSigma" << endl << MarginalSigma << endl;
}

// Compute information gain, mahalanobis distance, and relative distance covariance of a link vector
void CFlashFilter::complete_candidates_list(vector<Link>& links, const MatrixXd& R) const
{
  for (uint i = 0; i < links.size(); i++)
  {
    if (is_previous_pose(links.at(i).step, 1))
      complete_candidate(links.at(i), lastQ_);
    else
      complete_candidate(links.at(i), R);
  }
}

// Return the most information gain link of a given vector of links and removes it from the vector
Link CFlashFilter::extract_max_information_gain(vector<Link>& links) const
{ 
  uint idxMax = distance(links.begin(), max_element(links.begin(), links.end()));
  Link linkMax = links.at(idxMax);
  links.erase(links.begin() + idxMax);
  
  return linkMax;
}

double CFlashFilter::mahalanobis_distance(const VectorXd& mean, const MatrixXd& S, const VectorXd& d) const
{ 
  return sqrt((d - mean).transpose() * S.inverse() * (d - mean));
}

// GETs
uint CFlashFilter::get_nSteps() const 
{
  return PF_.get_nSteps();
}

uint CFlashFilter::get_nStates() const 
{
  return PF_.get_nStates();
}

SparseMatrix<double,ColMajor> CFlashFilter::get_lambda() const 
{
  return lambda_;
}

VectorXd CFlashFilter::get_mu() const
{
  return mu_;
}

VectorXd CFlashFilter::get_eta() const
{
  return eta_;
}

CPoseFilter CFlashFilter::get_PF() const
{
  return PF_;
}

MatrixXd CFlashFilter::get_F() const
{
  return F_;
}

vector<CPoseData> CFlashFilter::get_PD() const
{
  return PD_;
}

CPoseData CFlashFilter::get_PD(const uint& step) const
{
  if (PF_.step_2_state(step) != -1) 
    return PD_.at(PF_.step_2_state(step));
  else 
    printf("ERROR: get Pose Data of a redundant step!\n");
  return CPoseData();
}

// Cross covariance of a given pose with the current one. Returns the croos covariance of the pose at time slice 'step' with the current robot's pose
MatrixXd CFlashFilter::get_cross_covariance_with_pose(const uint& step) const
{  
  MatrixXd CC;
  
  
  if (step < PF_.get_nSteps() - 1)
  {
    if (PF_.step_2_state(step) == -1) 
      printf("ERROR: get cross covariance with a redundant step\n");
    CC = PD_.at(PF_.step_2_state(step)).get_CS() * F_;
  }
  else
    CC = get_pose_covariance(step);
  
  return CC;
}

// Marginal covariance between the current pose and a previous one. Returns the joint marginal covariance between the current pose and the pose at 'step'.
MatrixXd CFlashFilter::get_marginal_covariance(const uint& step) const
{
  // Output matrix is organized such that:
  //  - Top left sub-matrix corresponds to the marginal covariance of the current robot's pose
  //  - Bottom right one to marginal covariance of the at time 'step'.
  
  CGaussian Pi(get_pose_estimation(step));
  CGaussian Pn(get_pose_estimation(PF_.get_nSteps() - 1));
  
  uint dim = PF_.get_dim();
  MatrixXd S(2 * dim, 2 * dim);

  S.block(0, 0, dim, dim) = Pn.get_S();
  S.block(dim, dim, dim, dim) = Pi.get_S();
  S.block(dim, 0, dim, dim) = get_cross_covariance_with_pose(step);
  S.block(0, dim, dim, dim) = (S.block(dim, 0, dim, dim)).transpose();
  
  return S;
}

// Estimation of a given pose. Returns a Gaussian with the mean and marginal covariance for a the pose obtained at time 'step'.
CGaussian CFlashFilter::get_pose_estimation(const uint& step) const
{
  if (PF_.step_2_state(step) == -1) printf("ERROR: get pose estimation of a redundant step\n");
  return PD_.at(PF_.step_2_state(step)).get_gaussian();
}
  
// Covariance estimation of a given pose. Return the marginal covariance for the pose obtained at time 'step'
MatrixXd CFlashFilter::get_pose_covariance(const uint& step) const
{
  if (PF_.step_2_state(step) == -1) printf("ERROR: get pose covariance of a redundant step\n");
  return PD_.at(PF_.step_2_state(step)).get_S();
}

// Mean estimation of a given pose. Return the mean for the pose obtained at time 'step'
CPose CFlashFilter::get_pose_mean(const uint& step) const
{
  if (PF_.step_2_state(step) == -1) printf("ERROR: get pose mean of a redundant step\n");
  return PD_.at(PF_.step_2_state(step)).get_mean();
}

bool CFlashFilter::is_previous_pose(const uint& step, const uint& ignorePrevious) const
{
  return ((PF_.get_nStates() - 1) - PF_.step_2_state(step) <= ignorePrevious);
}

// DISTANCES

// Relative displacement between the current pose and a previous one.
vector<CGaussian> CFlashFilter::get_relative_displacement_2_pose(const uint& step) const
{
  // Returns a set of Gaussians representing the 1st order error approx. of the displacement from the current pose to the pose at time 'step'. 
  // The output includes one Gaussian for each dimension of the displacement between poses.
  
  CPose PoseN(get_pose_mean(get_nSteps()-1));
  CPose PoseI(get_pose_mean(step));
  
  // First order moment of the estimation of the relative distance from pn to pi in the frame of Pn
  VectorXd m(PoseN.absolute_2_relative(PoseI).get_vector());
  
  MatrixXd MarginalSigma = get_marginal_covariance(step);
  
  Jacobians H = PoseI.absolute_2_relative_jacobian(PoseN);
  MatrixXd Hni = concatenate_matrices(H.J2, H.J1, true);
  
  return get_general_distance_2_pose(m, MarginalSigma, Hni); 
}

// Distance from a pose the the current pose.
vector<CGaussian> CFlashFilter::get_general_distance_2_pose(const VectorXd& m, const MatrixXd& MS, const MatrixXd& H) const
{
  // This is the common part of 'GetDistance2Pose' and 'GetRelativeDistance2Pose'.
  
  const uint dim = PF_.get_dim();
  MatrixXd Sigma = H * MS * H.transpose();
  
  // Marginalize for each dimension
  vector<CGaussian> Gd(dim);
  for (uint i = 0; i < dim; i++)
  {
    VectorXd mv(1);
    mv(0) = m(i);
    MatrixXd ms(1,1);
    ms(0,0) = Sigma(i, i);
    Gd.at(i) = CGaussian(mv, ms);;
  }
  
  return Gd;
}

double CFlashFilter::prob_distance_2_pose_below_threshold(const VectorXd& t, const vector<CGaussian>& Gd) const
{
  // Checks if pose 'step' is close enought to the current pose.
  // Computes the probablity distribution on the relative displacement between the current robot's pose (stored in F_) and the pose obtained at time 'step'.
  // Gd is a one-dimensional distribution for each one of the components of the displacement. 
  // This function integrates this distributions in the interval [-t(i), t(i)] for each dimension 'i' and returns the minimum of all integrals.

  // This is equivalent to ProbDistance2PoseDataBelowThreshold but works single poses and not on sets of poses.

  double pIn = Gd.at(0).gaussian_CPD(t(0)) - Gd.at(0).gaussian_CPD(-t(0));
  
  for (uint i = 1; i < PF_.get_dim(); i++) 
    pIn = min(pIn, Gd.at(i).gaussian_CPD(t(i)) - Gd.at(i).gaussian_CPD(-t(i)));
  
  return pIn;
}

pair<double,double> CFlashFilter::prob_distance_2_posedata_above_threshold(const CPoseData& pd, const VectorXd& t,const double& pt) const
{
  //function [sLow sUp]=ProbDistance2PoseDataAboveThreshold(pd,F_,t,pt)

  // Computes the probability of the relative displacement between the current robot's pose and a given PoseData, pd, to be lower than 't'.
  // Note that the threshold 't' has one entry for each dimension of the pose space.
  //
  // This function integrates GetRelativeDistance2PoseData and ProbDistance2PoseDataBelowThreshold in order to perform a short-circuit
  // evaluation of the conditions for all the dimensions of the relative displacement. 
  //
  // The probability is estimated as a range [sLow sUp]. The evaluation is performed while sUp is above the given threshold 'pt': as soon sUp is below 'pt' for any dimension, 
  // we know for sure that the tested pose is too far (at least in that dimension) from the current pose and it does not need to be used for sensor registration.
  // If after all the evaluations sUp is above 'st' all the poses in the PoseData object are neighbours of the current pose (nor further processing is needed).
  // If sUp is below 'pt' none of the poses represented in PoseData are neighbours. If 'pt' is inside the range [sLow,sUp], the elements in PoseData need to be further refined.
  //
  // For the special case of orientations, intervals in PoseData larger than 2*pi are directly produce [sLow sUp]=[0,1] meaning that they have to be further refined.
  // Note that for PoseData objets including a single pose sLow=sUp.
  
  double sUp;
  double sLow;
  
  const uint dim = PF_.get_dim(); 
  
  if (!pd.is_interval())
  {
    sUp = prob_distance_2_pose_below_threshold(t, get_relative_displacement_2_pose(pd.get_id())); 
    sLow = sUp;
  }
  else if (dim==3)
  {
    CPose PoseN(get_pose_mean(get_nSteps() - 1)); 
    MatrixXd SN = get_pose_covariance(get_nSteps() - 1); 
    MatrixXd FF = get_F(); 
    CInterval CS = pd.get_interval_CS();
    CInterval S = pd.get_interval_S();
    
    double o1 = PoseN.get_vector()(2);
    double c1 = cos(o1);
    double s1 = sin(o1);
    double s1c1 = s1 * c1;
    double s12 = pow(s1,2);
    double c12 = pow(c1,2);
    
    MatrixXd R = MatrixXd::Identity(3, 3);
    R(0,0) = c1;
    R(0,1) = s1;
    R(1,0) = -s1;
    R(1,1) = c1;
    
    MatrixXd T1 = MatrixXd::Zero(3, 3);
    T1(0,2) = s1;
    T1(1,2) = c1;
    
    MatrixXd T2 = MatrixXd::Zero(3, 3);
    T2(0,2) = -c1;
    T2(1,2) = s1;
    
    VectorXd A1 = (T1 * SN * T1.transpose()).diagonal();
    VectorXd A2 = (T2 * SN * T2.transpose()).diagonal();
    VectorXd A3 = 2 * (T1 * SN * T2.transpose()).diagonal();
    VectorXd A4 = 2 * (R * SN * T1.transpose()).diagonal();
    VectorXd A5 = 2 * (R * SN * T2.transpose()).diagonal();
    VectorXd A6 = (R * SN * R.transpose()).diagonal();
    
    CInterval dx = pd.get_interval_mean()(0) - PoseN.get_vector()(0);
    CInterval dy = pd.get_interval_mean()(1) - PoseN.get_vector()(1);
    CInterval dx2 = dx.multiply_element_wise(dx); //power_element_wise(2);
    CInterval dy2 = dy.multiply_element_wise(dy); //power_element_wise(2);
    CInterval dxdy = dx.multiply_element_wise(dy);
    
    CInterval ct1 = CS(0,2) * s1c1;
    CInterval B1_1 = -ct1 + CS(1,2) * (-s12);
    CInterval ct2 = CS(1,2) * s1c1;
    CInterval B2_1 = ct2 + CS(0,2) * c12;
    CInterval ct3 = (CS(0,1) + CS(1,0)) * s1c1;
    CInterval B3_1 = CS(0,0) * c12 + CS(0,2) * (FF(2, 1) * s1c1 + FF(2, 0) * c12) + ct3 + CS(1,1) * s12 + CS(1,2) * (FF(2, 1) * s12 + FF(2, 0) * s1c1);
    CInterval ct4 = S(0,1) * (2 * s1c1);
    CInterval C1_1 = S(0,0) * c12 + ct4 + S(1,1) * s12;
    CInterval ct5 = (B3_1 * (-2) + C1_1) + A6(0);
    CInterval ct6 = (B1_1 * 2) + A4(0);
    CInterval ct7 = (B2_1 * 2) + A5(0);
    CInterval S1 = dx2 * A1(0) + dy2 * A2(0) + dxdy * A3(0) + ct6.multiply_element_wise(dx) + ct7.multiply_element_wise(dy) + ct5;
    
    CInterval Sx = S1.positive();
    CInterval m_1 = dx * c1 + dy * s1;
    
    CInterval rtU = ( t(0) - m_1).divide_element_wise((Sx * 2).sqrt_element_wise());
    CInterval rtL = (-t(0) - m_1).divide_element_wise((Sx * 2).sqrt_element_wise());
    
    sUp = min(0.5 * (erf(rtU.get_upper()(0,0)) - erf(rtL.get_lower()(0,0))), 1.0);
    sLow = max(0.5 * (erf(rtU.get_lower()(0,0)) - erf(rtL.get_upper()(0,0))), 0.0);
    
    if (sUp>pt)
    {
      CInterval B1_2 = ct1 + CS(1,2) * (-c12);
      CInterval B2_2 = -ct2 + CS(0,2) * s12;
      CInterval B3_2 = CS(0,0) * s12 + CS(0,2) * (-(FF(2, 1) * s1c1 - FF(2, 0) * s12)) - ct3 + CS(1,1) * c12 + CS(1,2) * (FF(2, 1) * c12 - FF(2, 0) * s1c1);
      CInterval C1_2 = S(0,0) * s12 -ct4 + S(1,1) * c12;
      CInterval ct8 = (B3_2 * (-2) + C1_2) + A6(1);
      CInterval ct9 = B1_2 * 2 + A4(1);
      CInterval ct10 = B2_2 * 2 + A5(1);
      CInterval S2 = dx2 * A1(1) + dy2 * A2(1) + dxdy * A3(1) + ct9.multiply_element_wise(dx) + ct10.multiply_element_wise(dy) + ct8;

      CInterval Sy = S2.positive(); 
      CInterval m_2 = dx * (-s1) + dy * c1;

      CInterval rtU = (t(1) - m_2).divide_element_wise((Sy * 2).sqrt_element_wise());
      CInterval rtL = (-t(1) - m_2).divide_element_wise((Sy * 2).sqrt_element_wise());
      
      sUp = min(min(0.5 * (erf(rtU.get_upper()(0,0)) - erf(rtL.get_lower()(0,0))), 1.0), sUp);
      sLow = min(max(0.5 * (erf(rtU.get_lower()(0,0)) - erf(rtL.get_upper()(0,0))), 0.0), sLow);
      
      if (sUp>pt)
      {
        if (pd.get_interval_mean().diameter(2) <= 1.5 * M_PI)
        {
          CInterval S3 = (dx2 * A1(2) + dy2 * A2(2) + dxdy * A3(2) + dx * A4(2) + dy * A5(2) + CS(2,2) * (-2) + S(2,2)) + A6(2);
          CInterval So = S3.positive();
          
          CInterval d_o(pd.get_interval_mean()(2) - PoseN.get_vector()(2));
          d_o = d_o.pi_2_pi(d_o);
          
          CInterval rtU = ( t(2) - d_o).divide_element_wise((2 * So).sqrt_element_wise());
          CInterval rtL = (-t(2) - d_o).divide_element_wise((2 * So).sqrt_element_wise());
    
          sUp = min(min(0.5 * (erf(rtU.get_upper()(0,0)) - erf(rtL.get_lower()(0,0))), 1.0), sUp);
          sLow = min(max(0.5 * (erf(rtU.get_lower()(0,0)) - erf(rtL.get_upper()(0,0))), 0.0), sLow);
        }
        else
        {
          sUp=1;
          sLow=0;
        }
      }
    }
  }
  
  pair<double,double> result(sLow, sUp);
  return result;
} 

// MATH OPERATIONS

// returns a sparse matrix block given a sparse matrix and indexes
SparseMatrix<double,ColMajor> CFlashFilter::sparse_block(const SparseMatrix<double,ColMajor>& sm, const uint& row, const uint& Nrows, const uint& col, const uint& Ncols) const
{
  SparseMatrix<double,RowMajor> smcols;
  smcols = sm.middleCols(col, Ncols); //sm is a column major sparse matrix
  SparseMatrix<double,ColMajor> smblock;
  smblock = smcols.middleRows(row,Nrows); //smcols is a row major sparse matrix
  
  return smblock;
}

// inserts the sparse matrix 'ins' into the sparse matrix 'original' in the place given by 'row' and 'col' integers
void CFlashFilter::insert_sparse_block(const SparseMatrix<double>& ins, SparseMatrix<double>& original, const uint& row, const uint& col) const
{
  // NOTE: it erase the values of 'original' where 'ins' is inserted
  erase_sparse_block(original, row, ins.rows(), col, ins.cols());
  
  for (int k=0; k<ins.outerSize(); ++k)
    for (SparseMatrix<double>::InnerIterator iti(ins,k); iti; ++iti)
      original.coeffRef(iti.row() + row, iti.col() + col) = iti.value();

  original.makeCompressed();
}

void CFlashFilter::erase_sparse_block(SparseMatrix<double>& original, const uint& row, const uint& Nrows, const uint& col, const uint& Ncols) const
{
  for (uint i = row; i < row + Nrows; i++)
    for (uint j = col; j < row + Ncols; j++)
      original.coeffRef(i,j) = 0.0;
}

// void CFlashFilter::add_sparse_block(const SparseMatrix<double>& ins, SparseMatrix<double>& original, const uint& row, const uint& col)
// {
//   for (int k=0; k<ins.outerSize(); ++k)
//   {
//     for (SparseMatrix<double>::InnerIterator iti(ins,k); iti; ++iti)
//       original.coeffRef(iti.row() + row, iti.col() + col) += iti.value();
//   }
// }

void CFlashFilter::add_sparse_block(const MatrixXd& ins, SparseMatrix<double>& original, const uint& row, const uint& col)
{
  for (uint r=0; r<ins.rows(); ++r)
    for (uint c = 0; c < ins.cols(); c++)
      original.coeffRef(r + row, c + col) += ins(r,c);
}

// concatenate dense matrices, horizontal if 'horitzontal' is true or vertical else
MatrixXd CFlashFilter::concatenate_matrices(const MatrixXd& M1,const MatrixXd& M2, const bool& horizontal) const
{
  const uint Nrows1 = M1.rows();
  const uint Nrows2 = M2.rows();
  const uint Ncols1 = M1.cols();
  const uint Ncols2 = M2.cols();
  MatrixXd M;
  
  if (horizontal)
  {
    MatrixXd MH(Nrows1,Ncols1+Ncols2);
    MH.block(0, 0, Nrows1, Ncols1) = M1;
    MH.block(0, Ncols1, Nrows2, Ncols2) = M2;
    M=MH;
  }
  else
  {
    MatrixXd MV(Nrows1+Nrows2,Ncols1);
    MV.block(0, 0, Nrows1, Ncols1) = M1;
    MV.block(Nrows1, 0, Nrows2, Ncols2) = M2;
    M=MV;
  } 
  return M;
}

// Check if matrix M is Semi Positive Definite
bool CFlashFilter::is_semi_pos_def(const MatrixXd& M) const
{
  EigenSolver<MatrixXd> es(M);
  //VectorXcd eigvals = M.eigenvalues();

  if ((es.eigenvalues().real().array() < -1e-12).any() || !es.eigenvalues().imag().isZero())
  {
    //cout << "Not Semipositive definite!"<< endl << es.eigenvalues() << endl;
    // MatrixXd EIGS = MatrixXd((es.eigenvalues().real().array() >= 0).select(es.eigenvalues().real(),0).asDiagonal()); //MatrixXd(es.eigenvalues().real().array().abs().matrix().asDiagonal());
    // cout << "New forced eigenvalues"<< endl << EIGS << endl;
    // cout << "Original matrix"<< endl << M << endl;
    // M = es.eigenvectors().real() * EIGS * es.eigenvectors().real().inverse();
    // cout << "Semidefinite positive forced matrix"<< endl << M << endl;
    // cout << "New matrix eigenvalues"<< endl << M.eigenvalues() << endl;

    return false;
  }
  else
    return true;
}

// all angles of mu into the (-PI, PI] interval
void CFlashFilter::angle_correction_mu(VectorXd m)
{
  for (uint i = 0; i < m.size() / 3; i++)
  {
    while (m(3 * i + 2) > M_PI)
      m(3 * i + 2) -= 2 * M_PI;
    while (m(3 * i + 2) <-M_PI)
      m(3 * i + 2) += 2 * M_PI;
  }
}

void CFlashFilter::make_symmetric(MatrixXd& M) const
{
  M = (M + M.transpose()) / 2;
  if (M != M.transpose())
    for (uint row = 0; row < M.rows(); row++)
      for (uint col = row; col < M.cols(); col++)
        M(col,row) = M(row,col);
}

void CFlashFilter::make_semi_pos_def(MatrixXd& M) const
{
  EigenSolver<MatrixXd> es(M);

  if ((es.eigenvalues().real().array() < -1e12).any() || !es.eigenvalues().imag().isZero())
  {
    //cout << "Old matrix eigenvalues"<< endl << M.eigenvalues() << endl;
    MatrixXd EIGS = MatrixXd((es.eigenvalues().real().array() >= 0).select(es.eigenvalues().real(),0).asDiagonal()); //MatrixXd(es.eigenvalues().real().array().abs().matrix().asDiagonal());
    // cout << "New forced eigenvalues"<< endl << EIGS << endl;
    // cout << "Original matrix"<< endl << M << endl;
    M = es.eigenvectors().real() * EIGS * es.eigenvectors().real().inverse();
    // cout << "Semidefinite positive forced matrix"<< endl << M << endl;
    //cout << "New matrix eigenvalues"<< endl << M.eigenvalues() << endl;
  }
}

// UTILITIES
// Conversion from a sparse matrix to a dense one (multiply dense identity matrix). ONLY FOR PRINTING
MatrixXd CFlashFilter::sparse_2_dense(const SparseMatrix<double>& sm) const
{
  uint dim=sm.cols();
  MatrixXd id=MatrixXd::Identity(dim,dim);
  return sm*id;
}
