#ifndef _FLASHFILTER_H
#define _FLASHFILTER_H
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/SparseCholesky>
#include "poseFilter.h"
#include "poseData.h"
#include "interval.h"
#include <math.h>
// #include <assert.h>
// #include "ctime.h"

using namespace Eigen;
using namespace std;

/** \struct Link
 *  \brief stores information of a link
 */
struct Link
{
  double iGain;
  uint step;
  double pd;
  VectorXd d;
  MatrixXd S_d;
  bool operator< (const Link& L) const
  {
    return iGain < L.iGain;
  };
};

/*!

  \class CFlashFilter
  \brief Includes state augmentation and state update methods, the information 
  gain function and the rest of filter methods. It contains the overall 
  information matrix and information vector. A pose Filter and a pose Data store
  all filter poses.

  [1] V. Ila, J. M. Porta, and J. Andrade-Cetto; Information-based compact 
      Pose SLAM; IEEE Trans. Robot., vol. 26, no. 1, pp. 78-93, Feb. 2010.
  [2] J. Vallvé; Active Exploration in Pose SLAM; M.Thesis June 2013

*/
class CFlashFilter
{
  public:
    CFlashFilter();
    CFlashFilter(const CFlashFilter& FF);
    CFlashFilter(const CGaussian& gaussian);
    ~CFlashFilter();
    
    // METHODS
    /** \brief executes a state_augment
     * 
     * updates the generic pose filter
     * 
     * \param step          
     * \param d             pose
     * \param Q             pose's noise
     * \param overwritePose 
     */
    void state_augment(const uint& step,const CPose& d,const MatrixXd& Qd,const bool& overwritePose, const double& min_cov);
    
    /** \brief executes a state update
     * 
     * updates loop in the generic pose filter
     * 
     * \param step  The step to be linked with the current robot's pose to form 
                    a loop.       
     * \param d     Displacement between the current robot's pose and pose 'i' 
                    according to the sensors readings.          
     * \param R     Noise of the sensor
     * \return if the update sucteed
     */
    bool state_update(const uint& i,const CPose& d,MatrixXd& Q, const double& min_cov);
    
    /** \brief Calculates the information gain of a link with the current robot pose
     *
     * Information gain of stablishing a link between the current robot's 
     * and that at time 'step' with a sensor with noise R.
     * 
     * \param R    Noise of the sensor         
     * \param step                   
     */
    void complete_candidate(Link& link, const MatrixXd& R) const;
    
    /** \brief Compute information gains of links vector
     * 
     * \param L list of candidates
     * \param R Noise of the sensor
     * \return a vector of links
     */
    void complete_candidates_list(vector<Link>& links, const MatrixXd& R) const;

    /** \brief
     * 
     * \param step          
     * \param d             
     * \param Q             
     * \param overwritePose 
     */
    Link extract_max_information_gain(vector<Link>& links) const;

    /** \brief
     * 
     * \param step with which would close the loop
     * \param d distance measured
     * \return the mahalanobis distance of the measurement taking the relative covariance of the two poses
     */
    double mahalanobis_distance(const VectorXd& mean, const MatrixXd& S, const VectorXd& d) const;
    
    // GETs
    
    /** \brief
     * 
     * \return 
     */
    uint get_nSteps() const;
    
    /** \brief
     * 
     * \return 
     */
    uint get_nStates() const;
    
    /** \brief
     * 
     * \return 
     */
    SparseMatrix<double,ColMajor> get_lambda() const;
    
    /** \brief
     * 
     * \return 
     */
    VectorXd get_mu() const;
    
    /** \brief
     * 
     * \return 
     */
    VectorXd get_eta() const;
    
    /** \brief
     * 
     * \return 
     */
    CPoseFilter get_PF() const;
    
    /** \brief
     * 
     * \return 
     */
    MatrixXd get_F() const;
    
    /** \brief
     * 
     * \return 
     */
    vector<CPoseData> get_PD() const;
    
    /** \brief
     * 
     * \return 
     */
    CPoseData get_PD(const uint& step) const;
    
    
    /** \brief
     * 
     * \param  step 
     * \return      
     */
    MatrixXd get_cross_covariance_with_pose(const uint& step) const;
    
    /** \brief
     * 
     * \param  step 
     * \return      
     */
    MatrixXd get_marginal_covariance(const uint& step) const;
    
    /** \brief
     * 
     * \param  step 
     * \return      
     */
    CGaussian get_pose_estimation(const uint& step) const;
    
    /** \brief
     * 
     * \param  step 
     * \return      
     */
    MatrixXd get_pose_covariance(const uint& step) const;
    
    /** \brief
     * 
     * \param  step 
     * \return      
     */
    CPose get_pose_mean(const uint& step) const;
    
    /** \brief
     * 
     * \param  step 
     * \return      
     */
    bool is_previous_pose(const uint& step, const uint& ignorePrevious) const;
    
    // DISTANCES
    vector<CGaussian> get_relative_displacement_2_pose(const uint& step) const;
    vector<CGaussian> get_general_distance_2_pose(const VectorXd& m, const MatrixXd& MS, const MatrixXd& H) const;
    double prob_distance_2_pose_below_threshold(const VectorXd& t, const vector<CGaussian>& Gd) const;
    pair<double,double> prob_distance_2_posedata_above_threshold(const CPoseData& PD, const VectorXd& t,const double& pt) const;
    
    // MATH OPERATIONS
    SparseMatrix<double,ColMajor> sparse_block(const SparseMatrix<double,ColMajor>& sm,const uint& row,const uint& Nrows,const uint& col,const uint& Ncols) const;
    void insert_sparse_block(const SparseMatrix<double>& ins, SparseMatrix<double>& original, const uint& row, const uint& col) const;
    void erase_sparse_block(SparseMatrix<double>& original, const uint& row, const uint& Nrows, const uint& col, const uint& Ncols) const;
    void add_sparse_block(const MatrixXd& ins, SparseMatrix<double>& original, const uint& row, const uint& col);
    MatrixXd concatenate_matrices(const MatrixXd& M1, const MatrixXd& M2, const bool& horizontal) const;
    bool is_semi_pos_def(const MatrixXd& M) const;
    void angle_correction_mu(VectorXd m);
    void make_symmetric(MatrixXd& M) const;
    void make_semi_pos_def(MatrixXd& M) const;

    // UTILITIES
    /** \brief
     * 
     * \param  sm 
     * \return    
     */
    MatrixXd sparse_2_dense(const SparseMatrix<double>& sm) const;
    
  private:
    SparseMatrix<double,ColMajor> lambda_, secondLastMarginalLambda_;
    VectorXd mu_, eta_, secondLastEta_;
    vector<CPoseData> PD_;
    CPoseFilter PF_;
    MatrixXd F_, secondLastF_, lastQ_;

    // CTime t0_, t1_;
    double t_simplicial_, t_supernodal_;
};

#endif 
