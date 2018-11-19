#ifndef _POSESLAM_H
#define _POSESLAM_H
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/SparseCholesky>
#include "poseFilter.h"
#include "poseData.h"
#include "interval.h"
#include "flashFilter.h"
#include "btree.h"
#include <math.h>

/*! \struct Params
 *  \brief set of PoseSLAM parameters
 */
struct Params
{
  std::pair<double, double> pdRange;
  std::pair<double, double> igRange;
  Vector3d matchArea;
  MatrixXd LoopNoise;
  int ignorePrevious;
  double K_mahalanobis;
  double min_cov;
};

/*!
  \brief Efficient PoseSLAM algorithm

  The key of PoseSLAM is the use of the information form to represent probability distributions.
  To mantain a state of bounded size, it only keeps non-redundant poses and highly informative links

  [1] V. Ila, J. M. Porta, and J. Andrade-Cetto; Information-based compact Pose SLAM; IEEE Trans. Robot., vol. 26, no. 1, pp. 78-93, Feb. 2010.
  [2] J. Vallvé; Active Exploration in Pose SLAM; M.Thesis June 2013

*/
class CPoseSLAM
{
  public:
    CPoseSLAM(); //!< Empty constructor TODO ?
    CPoseSLAM(const MatrixXd& initNoise); //!< Constructor setting initial noise
    CPoseSLAM(const MatrixXd& initNoise, const Params& params); //!< Constructor setting initial noise and algorithm parameters
    CPoseSLAM(const VectorXd& initPose, const MatrixXd& initNoise); //!< Constructor setting initial pose and noise
    CPoseSLAM(const VectorXd& initPose, const MatrixXd& initNoise, const Params& params); //!< Constructor setting initial pose, noise and parameters
    CPoseSLAM(const CGaussian& gaussian); //!< Constructor setting gaussian filter
    CPoseSLAM(const CGaussian& gaussian, const Params& params); //!< Constructor setting gaussian filter and algorithm parameters
    CPoseSLAM(const CPoseSLAM& pSLAM); //!< Constructor copying data from another poseSLAM object 
    ~CPoseSLAM();
    
    // METHODS
    /**
     * \brief State Augmentation
     *
     * \param step RO step
     * \param d    RO displacement
     * \param Q    RO noise
     */
    void augmentation(const uint& step, const Vector3d& d, const MatrixXd& Q);
    /**
     * \brief Creation of the link candidates list
     */
    void create_candidates_list();
    /**
     * \brief Evaluation of link redundancy
     */
    void redundant_evaluation();
    /**
     * \brief Returns if a link is or not redundant
     * \return True if its a valid link
     */
    bool loop_closure_requeriments() const;
    /**
     * \brief checks the conditions to close the loop
     * \TODO what if the update can't be done? exceptions?
     * \param  LoopClosureNoise 
     * \param  LoopClosureD distance    
     * \return True if the loop closure has been successful
     */
    bool try_loop_closure(MatrixXd& LoopClosureNoise, const VectorXd& LoopClosureD);
    /**
     * \brief updates the list of candidates
     */
    void update_candidates_list();
    /**
     * \brief if there's a candidate in the list
     * \return True if exists
     */
    bool any_candidate() const;
    /**
     * \brief Sets the candidate choosing the one with higher information gain
     */
    void select_best_candidate();
    
    // SETs
    /**
     * \brief sets algorithm parameters
     * \param params
     */
    void set_parameters(const Params& params);
    /**
     * \brief sets if the algorithm is or not redundant
     * \param redundant
     */
    void set_redundant(const bool& redundant);
    
    // GETs
    /**
     * \brief gets the algorithm parameters
     * \return the struct of parameters
     */
    Params get_params() const;
    /**
     * \brief gets the Flash Filter object
     * \return Flash Filter
     */
    CFlashFilter get_FF() const;
    /**
     * \brief gets the binary tree
     * \return Binary Tree
     */
    CBtree get_BTree() const;
    /**
     * \brief says if a link is redundant
     * \return true if it is redundant
     */
    bool is_redundant() const;
    /**
     * \brief says if theres a loop closed
     * \return true if theres a closed loop
     */
    bool is_any_closed() const;
    /**
     * \brief get Links list
     * \return vector of links
     */
    std::vector<Link> get_LL() const;
    /**
     * \brief gets the candidate's link
     * \return only the link
     */
    Link get_candidate_link() const;
    /**
     * \brief gets the candidate's step
     * \return only the step
     */
    uint get_candidate_step() const;
    /**
     * \brief gets the trajectory
     * \return vector of poses
     */
    std::vector<VectorXd> get_trajectory() const;
    /**
     * \brief gets the trajectory covariances
     * \return vector of covariances
     */
    std::vector<MatrixXd> get_trajectory_covariance() const;
    /**
     * \brief gets the trajectory steps
     * \return steps in a vector of ints
     */
    std::vector<uint> get_trajectory_steps() const;
    
  private:
    Params Parameters_;        //!< Set of algorithm parameters
    CFlashFilter FFilter_;     //!< Algorithm's Flash Filter
    CBtree BTree_;             //!< Agorithm's Binary Tree
    bool redundantPose_;       //!< tells if a pose is redundant or not
    bool anyClosed_;           //!< tells if there is any loop closed
    std::vector<Link> LL_;     //!< List of possible candidates
    Link candidate_;  //!< Candidate for the state update
};

#endif 
