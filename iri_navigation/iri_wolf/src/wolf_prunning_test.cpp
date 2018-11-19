//ros dependencies
#include "ros/ros.h"
#include "ros/time.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

//C includes for sleep, time and main args
#include "unistd.h"

//std includes
#include <iostream>
#include <memory>
#include <random>
#include <cmath>
#include <queue>

//Wolf includes
#include "wolf/wolf_manager.h"
#include "wolf/capture_void.h"
#include "wolf/ceres_wrapper/ceres_manager.h"

void insertSparseBlock(const Eigen::SparseMatrix<WolfScalar>& ins, Eigen::SparseMatrix<WolfScalar>& original, const unsigned int& row, const unsigned int& col);

bool loadGraphFile(const std::string& file_path, WolfProblem* wolf_problem_full, WolfProblem* wolf_problem_prun, const int MAX_VERTEX, Eigen::SparseMatrix<WolfScalar>& Lambda, std::map<FrameBase*, unsigned int>& frame_ptr_2_index_prun);

void addEdgeToProblem(const Eigen::Vector3s& edge_vector, const Eigen::Matrix3s& edge_information, FrameBase* frame_old_ptr, FrameBase* frame_new_ptr);
void addEdgeToLambda(const Eigen::Matrix3s& edge_information, FrameBase* frame_old_ptr, FrameBase* frame_new_ptr, Eigen::SparseMatrix<WolfScalar>& Lambda, unsigned int& edge_old, unsigned int& edge_new);
void wolfToMarker(visualization_msgs::Marker& marker, WolfProblem* wolf_problem_ptr);

int main(int argc, char **argv)
{
    int max_vertex_;
    std::string file_path_;
    ceres::Solver::Summary summary_full, summary_prun;
    Eigen::MatrixXs Sigma_ii(3,3), Sigma_ij(3,3), Sigma_jj(3,3), Sigma_z(3,3), Ji(3,3), Jj(3,3);
    WolfScalar xi, yi, thi, si, ci, xj, yj;
    double t_sigma_ceres, t_sigma_manual, t_ig;
    WolfScalar ig_threshold_;
    ros::Time t1;

    visualization_msgs::Marker prior_marker_, prunned_solution_marker_, full_solution_marker_;
    prior_marker_.type = visualization_msgs::Marker::LINE_STRIP;
    prior_marker_.header.frame_id = "map";
    prior_marker_.pose.position.x = 0;
    prior_marker_.pose.position.x = 0;
    prior_marker_.pose.position.x = 0;
    prior_marker_.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    prior_marker_.scale.x = 0.1;
    prior_marker_.color.r = 1; //yellow
    prior_marker_.color.g = 1;
    prior_marker_.color.b = 0;
    prior_marker_.color.a = 1;
    prior_marker_.ns = "/prior";
    prior_marker_.id = 1;
    prunned_solution_marker_ = prior_marker_;
    prior_marker_.color.r = 1; //red
    prior_marker_.color.g = 0;
    prior_marker_.color.b = 0;
    prior_marker_.color.a = 1;
    prior_marker_.ns = "/prunned";
    prior_marker_.id = 2;
    full_solution_marker_ = prior_marker_;
    prior_marker_.color.r = 0; //blue
    prior_marker_.color.g = 0;
    prior_marker_.color.b = 1;
    prior_marker_.color.a = 1;
    prior_marker_.ns = "/full";
    prior_marker_.id = 3;

    // loading variables
    std::map<FrameBase*, unsigned int> frame_ptr_2_index_prun;

    // Wolf problem
    WolfProblem* wolf_problem_full = new WolfProblem();
    WolfProblem* wolf_problem_prun = new WolfProblem();
    Eigen::SparseMatrix<WolfScalar> Lambda(0,0);

    // Ceres wrapper
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    ceres::Problem::Options problem_options;
    problem_options.cost_function_ownership = ceres::TAKE_OWNERSHIP;
    problem_options.loss_function_ownership = ceres::TAKE_OWNERSHIP;
    problem_options.local_parameterization_ownership = ceres::TAKE_OWNERSHIP;
    CeresManager* ceres_manager_full = new CeresManager(wolf_problem_full, problem_options);
    CeresManager* ceres_manager_prun = new CeresManager(wolf_problem_prun, problem_options);

    //ros init
    ros::init(argc, argv, "wolf_prunnning");

    ros::NodeHandle nh("~");
    nh.param<std::string>("file_path", file_path_, "/home/jvallve/bags/graphs/input_M3500b_toro.graph");
    nh.param<int>("max_vertex", max_vertex_, 0);
    nh.param<double>("ig_threshold", ig_threshold_, 0);
    ros::Publisher prior_publisher = nh.advertise <visualization_msgs::Marker> ("prior",1);
    ros::Publisher prunned_solution_publisher = nh.advertise <visualization_msgs::Marker> ("prunned_solution",1);
    ros::Publisher full_solution_publisher = nh.advertise <visualization_msgs::Marker> ("full_solution",1);

    if (max_vertex_ == 0) max_vertex_ = 1e6;

    // LOAD GRAPH
    loadGraphFile(file_path_, wolf_problem_full, wolf_problem_prun, max_vertex_, Lambda, frame_ptr_2_index_prun);

    // STORE PRIOR
    wolfToMarker(prior_marker_, wolf_problem_prun);

    // ADD PRIOR
    FrameBase* first_frame_full = wolf_problem_full->getTrajectoryPtr()->getFrameListPtr()->front();
    FrameBase* first_frame_prun = wolf_problem_prun->getTrajectoryPtr()->getFrameListPtr()->front();
    CaptureFix* initial_covariance_full = new CaptureFix(TimeStamp(0), first_frame_full->getState(), Eigen::Matrix3s::Identity() * 0.01);
    CaptureFix* initial_covariance_prun = new CaptureFix(TimeStamp(0), first_frame_prun->getState(), Eigen::Matrix3s::Identity() * 0.01);
    first_frame_full->addCapture(initial_covariance_full);
    first_frame_prun->addCapture(initial_covariance_prun);
    initial_covariance_full->processCapture();
    initial_covariance_prun->processCapture();
    //std::cout << "initial covariance: constraint " << initial_covariance_prun->getFeatureListPtr()->front()->getConstraintFromListPtr()->front()->nodeId() << std::endl << initial_covariance_prun->getFeatureListPtr()->front()->getMeasurementCovariance() << std::endl;
    Eigen::SparseMatrix<WolfScalar> DeltaLambda(Lambda.rows(), Lambda.cols());
    insertSparseBlock((Eigen::Matrix3s::Identity() * 100).sparseView(), DeltaLambda, 0, 0);
    Lambda = Lambda + DeltaLambda;

    // BUILD SOLVER PROBLEM
    std::cout << "updating ceres..." << std::endl;
    ceres_manager_full->update();
    ceres_manager_prun->update();
    std::cout << "updated!" << std::endl;

    // COMPUTE COVARIANCES
    // whole covariance computation
    t1 = ros::Time::now();
    Eigen::SimplicialLLT<Eigen::SparseMatrix<WolfScalar>> chol(Lambda);  // performs a Cholesky factorization of A
    Eigen::MatrixXs Sigma = chol.solve(Eigen::MatrixXs::Identity(Lambda.rows(), Lambda.cols()));
    t_sigma_manual = (ros::Time::now() - t1).toSec();
    std::cout << "SimplicialLLT: " << t_sigma_manual << std::endl;
    //std::cout << "Lambda" << std::endl << Lambda << std::endl;
    //std::cout << "Sigma" << std::endl << Sigma << std::endl;

//    std::cout << "ceres is computing covariances..." << std::endl;
//    t1 = ros::Time::now();
//    ceres_manager_prun->computeCovariances(ALL);//ALL_MARGINALS
//    t_sigma_ceres = (ros::Time::now() - t1).toSec();
//    std::cout << "computed!" << std::endl;

    // INFORMATION GAIN
    t1 = ros::Time::now();
    ConstraintBaseList constraints;
    wolf_problem_prun->getTrajectoryPtr()->getConstraintList(constraints);
    for (auto c_it=constraints.begin(); c_it!=constraints.end(); c_it++)
    {
        if ((*c_it)->getCategory() != CTR_FRAME) continue;

        // ii (old)
//        wolf_problem_prun->getCovarianceBlock((*c_it)->getFrameToPtr()->getPPtr(), (*c_it)->getFrameToPtr()->getPPtr(), Sigma_ii, 0, 0);
//        wolf_problem_prun->getCovarianceBlock((*c_it)->getFrameToPtr()->getPPtr(), (*c_it)->getFrameToPtr()->getOPtr(), Sigma_ii, 0, 2);
//        wolf_problem_prun->getCovarianceBlock((*c_it)->getFrameToPtr()->getOPtr(), (*c_it)->getFrameToPtr()->getPPtr(), Sigma_ii, 2, 0);
//        wolf_problem_prun->getCovarianceBlock((*c_it)->getFrameToPtr()->getOPtr(), (*c_it)->getFrameToPtr()->getOPtr(), Sigma_ii, 2, 2);
        Sigma_ii = Sigma.block<3,3>(frame_ptr_2_index_prun[(*c_it)->getFrameToPtr()]*3, frame_ptr_2_index_prun[(*c_it)->getFrameToPtr()]*3);
//        std::cout << "Sigma_ii" << std::endl << Sigma_ii << std::endl;

        // jj (new)
//        wolf_problem_prun->getCovarianceBlock((*c_it)->getCapturePtr()->getFramePtr()->getPPtr(), (*c_it)->getCapturePtr()->getFramePtr()->getPPtr(), Sigma_jj, 0, 0);
//        wolf_problem_prun->getCovarianceBlock((*c_it)->getCapturePtr()->getFramePtr()->getPPtr(), (*c_it)->getCapturePtr()->getFramePtr()->getOPtr(), Sigma_jj, 0, 2);
//        wolf_problem_prun->getCovarianceBlock((*c_it)->getCapturePtr()->getFramePtr()->getOPtr(), (*c_it)->getCapturePtr()->getFramePtr()->getPPtr(), Sigma_jj, 2, 0);
//        wolf_problem_prun->getCovarianceBlock((*c_it)->getCapturePtr()->getFramePtr()->getOPtr(), (*c_it)->getCapturePtr()->getFramePtr()->getOPtr(), Sigma_jj, 2, 2);
        Sigma_jj = Sigma.block<3,3>(frame_ptr_2_index_prun[(*c_it)->getCapturePtr()->getFramePtr()]*3, frame_ptr_2_index_prun[(*c_it)->getCapturePtr()->getFramePtr()]*3);
//        std::cout << "Sigma_jj" << std::endl << Sigma_jj << std::endl;

        // ij (old-new)
//        wolf_problem_prun->getCovarianceBlock((*c_it)->getFrameToPtr()->getPPtr(), (*c_it)->getCapturePtr()->getFramePtr()->getPPtr(), Sigma_ij, 0, 0);
//        wolf_problem_prun->getCovarianceBlock((*c_it)->getFrameToPtr()->getPPtr(), (*c_it)->getCapturePtr()->getFramePtr()->getOPtr(), Sigma_ij, 0, 2);
//        wolf_problem_prun->getCovarianceBlock((*c_it)->getFrameToPtr()->getOPtr(), (*c_it)->getCapturePtr()->getFramePtr()->getPPtr(), Sigma_ij, 2, 0);
//        wolf_problem_prun->getCovarianceBlock((*c_it)->getFrameToPtr()->getOPtr(), (*c_it)->getCapturePtr()->getFramePtr()->getOPtr(), Sigma_ij, 2, 2);
        Sigma_ij = Sigma.block<3,3>(frame_ptr_2_index_prun[(*c_it)->getFrameToPtr()]*3, frame_ptr_2_index_prun[(*c_it)->getCapturePtr()->getFramePtr()]*3);
//        std::cout << "Sigma_ij" << std::endl << Sigma_ij << std::endl;

        //jacobian
        xi = *(*c_it)->getFrameToPtr()->getPPtr()->getPtr();
        yi = *((*c_it)->getFrameToPtr()->getPPtr()->getPtr()+1);
        thi = *(*c_it)->getFrameToPtr()->getOPtr()->getPtr();
        si = sin(thi);
        ci = cos(thi);
        xj = *(*c_it)->getCapturePtr()->getFramePtr()->getPPtr()->getPtr();
        yj = *((*c_it)->getCapturePtr()->getFramePtr()->getPPtr()->getPtr()+1);

        Ji << -ci,-si,-(xj-xi)*si+(yj-yi)*ci,
               si,-ci,-(xj-xi)*ci-(yj-yi)*si,
                0,  0,                    -1;
        Jj <<  ci, si, 0,
              -si, ci, 0,
                0,  0, 1;

        // Measurement covariance
        Sigma_z = (*c_it)->getFeaturePtr()->getMeasurementCovariance();
        //std::cout << "Sigma_z" << std::endl << Sigma_z << std::endl;

        // Information gain
        WolfScalar IG = 0.5 * log( Sigma_z.determinant() / (Sigma_z - (Ji * Sigma_ii * Ji.transpose() + Jj * Sigma_jj * Jj.transpose() + Ji * Sigma_ij * Jj.transpose() + Jj * Sigma_ij.transpose() * Ji.transpose())).determinant() );
        //std::cout << "IG = " << IG << std::endl;

        if (IG < 2)
            (*c_it)->setStatus(CTR_INACTIVE);
    }
    t_ig = (ros::Time::now() - t1).toSec();
    std::cout << "manual sigma computation " << t_sigma_manual << "s" << std::endl;
    std::cout << "ceres sigma computation " << t_sigma_ceres << "s" << std::endl;
    std::cout << "information gain computation " << t_ig << "s" << std::endl;

    // SOLVING PROBLEMS
    std::cout << "solving..." << std::endl;
    summary_full = ceres_manager_full->solve(ceres_options);
    std::cout << summary_full.BriefReport() << std::endl;
    ceres_manager_prun->update();
    summary_prun = ceres_manager_prun->solve(ceres_options);
    std::cout << summary_prun.BriefReport() << std::endl;

    // PLOTING RESULTS
    wolfToMarker(prunned_solution_marker_, wolf_problem_prun);
    wolfToMarker(full_solution_marker_, wolf_problem_full);

    while ( ros::ok() )
    {
        prior_publisher.publish(prior_marker_);
        prunned_solution_publisher.publish(prunned_solution_marker_);
        full_solution_publisher.publish(full_solution_marker_);
    }

    delete wolf_problem_full; //not necessary to delete anything more, wolf will do it!
    std::cout << "wolf_problem_ deleted!" << std::endl;
    delete ceres_manager_full;
    delete ceres_manager_prun;
    std::cout << "ceres_manager deleted!" << std::endl;
    //End message
    std::cout << " =========================== END ===============================" << std::endl << std::endl;

    //exit program
    return 0;
}

// inserts the sparse matrix 'ins' into the sparse matrix 'original' in the place given by 'row' and 'col' integers
void insertSparseBlock(const Eigen::SparseMatrix<WolfScalar>& ins, Eigen::SparseMatrix<WolfScalar>& original, const unsigned int& row, const unsigned int& col)
{
  for (int k=0; k<ins.outerSize(); ++k)
    for (Eigen::SparseMatrix<WolfScalar>::InnerIterator iti(ins,k); iti; ++iti)
      original.coeffRef(iti.row() + row, iti.col() + col) = iti.value();

  original.makeCompressed();
}

// load graph from file
bool loadGraphFile(const std::string& file_path, WolfProblem* wolf_problem_full, WolfProblem* wolf_problem_prun, const int MAX_VERTEX, Eigen::SparseMatrix<WolfScalar>& Lambda, std::map<FrameBase*, unsigned int>& frame_ptr_2_index_prun)
{
    std::ifstream offLineFile;
    std::map<unsigned int, FrameBase*> index_2_frame_ptr_full;
    std::map<unsigned int, FrameBase*> index_2_frame_ptr_prun;
    Eigen::Vector3s edge_vector, vertex_pose;
    Eigen::Matrix3s edge_information;
    FrameBase* vertex_frame_ptr_full;
    FrameBase* vertex_frame_ptr_prun;
    frame_ptr_2_index_prun.clear();
    unsigned int state_size = 0;

    offLineFile.open(file_path.c_str(), std::ifstream::in);
    if (offLineFile.is_open())
    {
        std::string buffer;
        unsigned int j = 0;
        // Line by line
        while (std::getline(offLineFile, buffer))
        {
            //std::cout << "new line:" << buffer << std::endl;
            std::string bNum;
            unsigned int i = 0;

            // VERTEX
            if (buffer.at(0) == 'V')
            {
                while (buffer.at(i) != ' ') i++; //skip rest of VERTEX word
                while (buffer.at(i) == ' ') i++; //skip white spaces
                //vertex index
                while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                unsigned int vertex_index = atoi(bNum.c_str());
                bNum.clear();

                if (vertex_index <= MAX_VERTEX+1)
                {
                    while (buffer.at(i) == ' ') i++; //skip white spaces
                    // vertex pose
                    // x
                    while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                    vertex_pose(0) = atof(bNum.c_str());
                    bNum.clear();
                    while (buffer.at(i) == ' ') i++; //skip white spaces
                    // y
                    while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                    vertex_pose(1) = atof(bNum.c_str());
                    bNum.clear();
                    while (buffer.at(i) == ' ') i++; //skip white spaces
                    // theta
                    while (i < buffer.size() && buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                    vertex_pose(2) = atof(bNum.c_str());
                    bNum.clear();
                    // add frame to problem
                    vertex_frame_ptr_full = new FrameBase(TimeStamp(0), new StateBlock(vertex_pose.head(2)), new StateBlock(vertex_pose.tail(1)));
                    vertex_frame_ptr_prun = new FrameBase(TimeStamp(0), new StateBlock(vertex_pose.head(2)), new StateBlock(vertex_pose.tail(1)));
                    wolf_problem_full->getTrajectoryPtr()->addFrame(vertex_frame_ptr_full);
                    wolf_problem_prun->getTrajectoryPtr()->addFrame(vertex_frame_ptr_prun);
                    // store
                    index_2_frame_ptr_full[vertex_index] = vertex_frame_ptr_full;
                    index_2_frame_ptr_prun[vertex_index] = vertex_frame_ptr_prun;
                    frame_ptr_2_index_prun[vertex_frame_ptr_prun] = vertex_index;
                    // increment state size
                    state_size +=3;
                    //std::cout << "Added vertex! index: " << vertex_index << " nodeId: " << vertex_frame_ptr_prun->nodeId() << std::endl << "pose: " << vertex_pose.transpose() << std::endl;
                }
            }
            // EDGE
            else if (buffer.at(0) == 'E')
            {
                // Information matrix resize (once)
                if (Lambda.rows() == 0) Lambda.resize(state_size,state_size);
                while (buffer.at(i) != ' ') i++; //skip rest of EDGE word
                while (buffer.at(i) == ' ') i++; //skip white spaces
                //from
                while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                unsigned int edge_old = atoi(bNum.c_str());
                bNum.clear();
                while (buffer.at(i) == ' ') i++; //skip white spaces
                //to index
                while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                unsigned int edge_new = atoi(bNum.c_str());
                bNum.clear();

                if (edge_new <= MAX_VERTEX+1 && edge_old <= MAX_VERTEX+1 )
                {
                    while (buffer.at(i) == ' ') i++; //skip white spaces
                    // edge vector
                    // x
                    while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                    edge_vector(0) = atof(bNum.c_str());
                    bNum.clear();
                    while (buffer.at(i) == ' ') i++; //skip white spaces
                    // y
                    while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                    edge_vector(1) = atof(bNum.c_str());
                    bNum.clear();
                    while (buffer.at(i) == ' ') i++; //skip white spaces
                    // theta
                    while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                    edge_vector(2) = atof(bNum.c_str());
                    bNum.clear();
                    while (buffer.at(i) == ' ') i++; //skip white spaces
                    // edge covariance
                    // xx
                    while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                    edge_information(0,0) = atof(bNum.c_str());
                    bNum.clear();
                    //skip white spaces
                    while (buffer.at(i) == ' ') i++;
                    // xy
                    while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                    edge_information(0,1) = atof(bNum.c_str());
                    edge_information(1,0) = atof(bNum.c_str());
                    bNum.clear();
                    while (buffer.at(i) == ' ') i++; //skip white spaces
                    // yy
                    while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                    edge_information(1,1) = atof(bNum.c_str());
                    bNum.clear();
                    while (buffer.at(i) == ' ') i++; //skip white spaces
                    // thetatheta
                    while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                    edge_information(2,2) = atof(bNum.c_str());
                    bNum.clear();
                    while (buffer.at(i) == ' ') i++; //skip white spaces
                    // xtheta
                    while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                    edge_information(0,2) = atof(bNum.c_str());
                    edge_information(2,0) = atof(bNum.c_str());
                    bNum.clear();
                    while (buffer.at(i) == ' ') i++; //skip white spaces
                    // ytheta
                    while (i < buffer.size() && buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                    edge_information(1,2) = atof(bNum.c_str());
                    edge_information(2,1) = atof(bNum.c_str());
                    bNum.clear();
                    // add capture, feature and constraint to problem
                    addEdgeToProblem(edge_vector, edge_information, index_2_frame_ptr_full[edge_old], index_2_frame_ptr_full[edge_new]);
                    addEdgeToProblem(edge_vector, edge_information, index_2_frame_ptr_prun[edge_old], index_2_frame_ptr_prun[edge_new]);
                    addEdgeToLambda(edge_information, index_2_frame_ptr_prun[edge_old], index_2_frame_ptr_prun[edge_new], Lambda, edge_old, edge_new);
                    //std::cout << "Added edge! " << constraint_ptr_prun->nodeId() << " from vertex " << constraint_ptr_prun->getCapturePtr()->getFramePtr()->nodeId() << " to " << constraint_ptr_prun->getFrameToPtr()->nodeId() << std::endl;
                    //std::cout << "vector " << constraint_ptr_prun->getMeasurement().transpose() << std::endl;
                    //std::cout << "information " << std::endl << edge_information << std::endl;
                    //std::cout << "covariance " << std::endl << constraint_ptr_prun->getMeasurementCovariance() << std::endl;
                    j++;
                }
            }
            else
                assert("unknown line");
        }
        printf("\nGraph loaded!\n");
        return true;
    }
    else
    {
        printf("\nError opening file!\n");
        return false;
    }
}

void addEdgeToProblem(const Eigen::Vector3s& edge_vector, const Eigen::Matrix3s& edge_information, FrameBase* frame_old_ptr, FrameBase* frame_new_ptr)
{
    FeatureBase* feature_ptr = new FeatureBase(edge_vector, edge_information.inverse());
    CaptureVoid* capture_ptr = new CaptureVoid(TimeStamp(0), nullptr);
    frame_new_ptr->addCapture(capture_ptr);
    capture_ptr->addFeature(feature_ptr);
    ConstraintOdom2D* constraint_ptr = new ConstraintOdom2D(feature_ptr, frame_old_ptr);
    feature_ptr->addConstraintFrom(constraint_ptr);
}

void addEdgeToLambda(const Eigen::Matrix3s& edge_information, FrameBase* frame_old_ptr, FrameBase* frame_new_ptr, Eigen::SparseMatrix<WolfScalar>& Lambda, unsigned int& edge_old, unsigned int& edge_new)
{
    WolfScalar xi = *(frame_old_ptr->getPPtr()->getPtr());
    WolfScalar yi = *(frame_old_ptr->getPPtr()->getPtr()+1);
    WolfScalar thi = *(frame_old_ptr->getOPtr()->getPtr());
    WolfScalar si = sin(thi);
    WolfScalar ci = cos(thi);
    WolfScalar xj = *(frame_new_ptr->getPPtr()->getPtr());
    WolfScalar yj = *(frame_new_ptr->getPPtr()->getPtr()+1);
    Eigen::MatrixXs Ji(3,3), Jj(3,3);
    Ji << -ci,-si,-(xj-xi)*si+(yj-yi)*ci,
           si,-ci,-(xj-xi)*ci-(yj-yi)*si,
            0,  0,                    -1;
    Jj <<  ci, si, 0,
          -si, ci, 0,
            0,  0, 1;
    //std::cout << "Ji" << std::endl << Ji << std::endl;
    //std::cout << "Jj" << std::endl << Jj << std::endl;
    Eigen::SparseMatrix<WolfScalar> DeltaLambda(Lambda.rows(), Lambda.cols());
    insertSparseBlock((Ji.transpose() * edge_information * Ji).sparseView(), DeltaLambda, edge_old*3, edge_old*3);
    insertSparseBlock((Jj.transpose() * edge_information * Jj).sparseView(), DeltaLambda, edge_new*3, edge_new*3);
    insertSparseBlock((Ji.transpose() * edge_information * Jj).sparseView(), DeltaLambda, edge_old*3, edge_new*3);
    insertSparseBlock((Jj.transpose() * edge_information * Ji).sparseView(), DeltaLambda, edge_new*3, edge_old*3);
    Lambda = Lambda + DeltaLambda;
}

void wolfToMarker(visualization_msgs::Marker& marker, WolfProblem* wolf_problem_ptr)
{
    marker.points.clear();
    marker.header.stamp = ros::Time::now();
    geometry_msgs::Point point;
    for (auto fr_it = wolf_problem_ptr->getTrajectoryPtr()->getFrameListPtr()->begin();
              fr_it != wolf_problem_ptr->getTrajectoryPtr()->getFrameListPtr()->end();
              fr_it++) //runs the list of frames
    {
        point.x = *(*fr_it)->getPPtr()->getPtr();
        point.y = *((*fr_it)->getPPtr()->getPtr()+1);
        point.z = 0.;
        marker.points.push_back(point);
    }
}
