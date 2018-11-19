/*
 * personBHMIP.cpp
 *
 *  Created on: Jul 9, 2013
 *      Author: gferrer
 */
#include "scene_elements/person_bhmip.h"
#include <Eigen/Dense>
#include <iostream>

Cperson_bhmip::Cperson_bhmip(unsigned int id, Cperson_abstract::target_type person_target_type,
		 Cperson_abstract::force_type person_force_type, double _time_window) :
    Cperson_abstract(id,person_target_type, person_force_type),
    time_window_( _time_window ) , phi_var_( 0.8*0.8 )
{
	destinations_.reserve(15);
}

Cperson_bhmip::~Cperson_bhmip()
{

}

void Cperson_bhmip::add_pointV( SpointV_cov point, Cperson_abstract::filtering_method filter)
{
	SpointV_cov filtered_pose;
	if( filter != Cperson_abstract::No_filtering )
	{
		SpointV_cov pointV = point;//only positions, position_covariances and time_stamp
		double dx, dy;
		if ( !trajectory_.empty() )
		{
			dx = point.x - trajectory_.back().x;
			dy = point.y - trajectory_.back().y;
			double dt =  point.time_stamp - trajectory_.back().time_stamp ;
			//just to avoid singularities, ideally time stamps are well separated in time
			if ( point.time_stamp - trajectory_.back().time_stamp  < 0.001 ) dt = 0.1;
			pointV.vx = dx / dt;
			pointV.vy = dy / dt;
		}
		else
		{
			pointV.vx = 0;
			pointV.vy = 0;
		}

		//slide temporal window for all trajectories
		trajectory_.push_back(pointV);
		trajectory_windowing();

		//filters velocities
		switch( filter )
		{
		  case Cperson_abstract::Linear_regression_filtering:
			filtered_pose = filter_current_state_linear_regression( );
			break;
		  case Cperson_abstract::Bayes_filtering:
			filtered_pose = filter_current_state_linear_regression_bayes( );
		  	break;
		  case Cperson_abstract::Low_pass_linear_regression_filtering:
		  default :
			filtered_pose = low_pass_filter_current_state_linear_regression( );
			break;
		}


	}
	else
	{
		//no filter is required
		trajectory_.push_back(point);
		//slide temporal window for all trajectories
		trajectory_windowing();
		filtered_pose = trajectory_.back();
	}

	//update current positions and velocities according to filtering results
	diff_pointV_ = filtered_pose - current_pointV_ ;
	current_pointV_ = filtered_pose;
	desired_velocity_ = filtered_pose.v();
	now_ = filtered_pose.time_stamp;


	//precalculates data required for the BHMIP
	intention_precalculation();

	observation_update_ = true;
}


SpointV_cov Cperson_bhmip::filter_current_state_linear_regression( )
{


	//Discard a single and double pose trajectory --------------------------------------------
	if (trajectory_.size() <= 2 )
	{
		return trajectory_.back();
	}
	// First order Regression: y(t) ~ beta * x(t) = beta_1 + beta_2 * x(t) ----------------------------
	//vector and matrices initialization
	unsigned int window_elements = trajectory_.size();
	double initial_ts = trajectory_.front().time_stamp;
	Eigen::VectorXd vx ( window_elements ) , vy ( window_elements );
	Eigen::MatrixXd R (window_elements , 2 ); //1st order regression -> 2 coeffs
	for (unsigned int i = 0 ; i < window_elements ; i++)
	{
		vx ( i ) = trajectory_.at(i).vx;
		vy ( i ) = trajectory_.at(i).vy;
		R.row ( i ) << 1 , trajectory_.at(i).time_stamp - initial_ts;
	}
	//pseudo inverse of the R matrix ---------------------------------------------------------------
	Eigen::MatrixXd invR ( 2 , window_elements ) , sqrR ( 2 , 2 );
	sqrR = R.transpose() * R;
	invR = sqrR.inverse() * R.transpose() ;

	//linear regression ----------------------------------------------------------------------------
	Eigen::Vector2d  r_pred (1 , trajectory_.back().time_stamp - initial_ts ),vx_coef,vy_coef;
	//invR * v = beta -> parameter estimation
	vx_coef = invR * vx;
	vy_coef = invR * vy;
	SpointV_cov filtered_pose(
			trajectory_.back().x, //no filtering at position
			trajectory_.back().y,
			trajectory_.back().time_stamp,
			r_pred.dot ( vx_coef ),
			r_pred.dot ( vy_coef ) );
    filtered_pose.cov[0] = trajectory_.back().cov_xx();
    filtered_pose.cov[1] = trajectory_.back().cov_xy();
    filtered_pose.cov[4] = trajectory_.back().cov_xy();
    filtered_pose.cov[5] = trajectory_.back().cov_yy();


	//covariance of velocity prediction, with respect to regression velocity
	Eigen::VectorXd vx_mean ( window_elements ) , vy_mean ( window_elements );
	vx_mean = R*vx_coef;
	vy_mean = R*vy_coef;
	double cov_vx = (vx - vx_mean).dot( vx - vx_mean );
	double cov_vy = (vy - vy_mean).dot( vy - vy_mean );
	double cov_vxy = (vy - vy_mean).dot( vx - vx_mean );

	//regression covariances + projected uncertainty from positions P_x / dt
	filtered_pose.cov[10] = cov_vx + 0.001;
	filtered_pose.cov[11] = cov_vxy;
	filtered_pose.cov[14] = filtered_pose.cov[11];
	filtered_pose.cov[15] = cov_vy + 0.001;

	return filtered_pose;
}


SpointV_cov Cperson_bhmip::filter_current_state_linear_regression_bayes( )
{

	//Discard a single and double pose trajectory --------------------------------------------
	if (trajectory_.size() <= 2 )
	{
		return trajectory_.back();
	}
	// First order Regression: y(t) ~ beta * x(t) = beta_1 + beta_2 * x(t) ----------------------------
	//vector and matrices initialization
	unsigned int window_elements = trajectory_.size();
	double initial_ts = trajectory_.front().time_stamp;
	Eigen::VectorXd vx ( window_elements ) , vy ( window_elements );
	Eigen::MatrixXd R (window_elements , 2 ); //1st order regression -> 2 coeffs
	for (unsigned int i = 0 ; i < window_elements ; i++)
	{
		vx ( i ) = trajectory_.at(i).vx;
		vy ( i ) = trajectory_.at(i).vy;
		R.row ( i ) << 1 , trajectory_.at(i).time_stamp - initial_ts;
	}
	//pseudo inverse of the R matrix ---------------------------------------------------------------
	Eigen::MatrixXd invR ( 2 , window_elements ) , sqrR ( 2 , 2 );
	sqrR = R.transpose() * R;
	invR = sqrR.inverse() * R.transpose() ;

	//linear regression ----------------------------------------------------------------------------
	Eigen::Vector2d  r_pred (1 , trajectory_.back().time_stamp - initial_ts ),vx_coef,vy_coef;
	//invR * v = beta -> parameter estimation
	vx_coef = invR * vx;
	vy_coef = invR * vy;
	SpointV_cov filtered_pose(
			trajectory_.back().x, //no filtering at position
			trajectory_.back().y,
			trajectory_.back().time_stamp,
			r_pred.dot ( vx_coef ),
			r_pred.dot ( vy_coef ) );
    filtered_pose.cov[0] = trajectory_.back().cov_xx();
    filtered_pose.cov[1] = trajectory_.back().cov_xy();
    filtered_pose.cov[4] = trajectory_.back().cov_xy();
    filtered_pose.cov[5] = trajectory_.back().cov_yy();

	//covariance of velocity prediction, average of uncertainty propagated from estimated positions, equidistribited
	double dt;
	Eigen::Matrix2d v_cov_joint = Eigen::Matrix2d::Zero();
	for( unsigned int i = 0; i < window_elements ; i++)
	{
		if( i == 0 )
			dt = R(1,1)-R(0,1);
		else
			dt = R(i,1)-R(i-1,1);
		Eigen::Matrix2d v_cov;
		v_cov << trajectory_.at(i).cov_xx() , trajectory_.at(i).cov_xy(),
				trajectory_.at(i).cov_xy() , trajectory_.at(i).cov_yy();
		v_cov_joint += v_cov / (dt*dt);
	}

	//regression covariances + projected uncertainty from positions P_x / dt
	double N = 2.0*(double)window_elements*(double)window_elements;
	filtered_pose.cov[10] = v_cov_joint(0,0)/N;
	filtered_pose.cov[11] = v_cov_joint(0,1)/N;
	filtered_pose.cov[14] = filtered_pose.cov[11];
	filtered_pose.cov[15] = v_cov_joint(1,1)/N;

	return filtered_pose;
}

SpointV_cov Cperson_bhmip::low_pass_filter_current_state_linear_regression( )
{

	//Discard a single and double pose trajectory --------------------------------------------
	if (trajectory_.size() <= 2 )
	{
		return trajectory_.back();
	}
	// First order Regression: y(t) ~ beta * x(t) = beta_1 + beta_2 * x(t) ----------------------------
	//vector and matrices initialization
	unsigned int window_elements = trajectory_.size();
//	double initial_ts = trajectory_.front().time_stamp;
	Eigen::VectorXd vx ( window_elements ) , vy ( window_elements );
	Eigen::VectorXd R (window_elements ); //0st order regression -> 1 coeffs
	Eigen::VectorXd r_pred (window_elements);
	for (unsigned int i = 0 ; i < window_elements ; i++)
	{
		vx ( i ) = trajectory_.at(i).vx;
		vy ( i ) = trajectory_.at(i).vy;
		R(i)=1;
		r_pred(i)=1;
	}
	//pseudo inverse of the R matrix ---------------------------------------------------------------
	Eigen::VectorXd invR (  window_elements );
	double sqrR =  R.dot( R);
	invR = (1/sqrR) * R.transpose() ;

	//linear regression ----------------------------------------------------------------------------

	//invR * v = beta -> parameter estimation
	double vx_coef =  invR.dot(vx.transpose());
	double vy_coef = invR.dot(vy.transpose());

	SpointV_cov filtered_pose(
			trajectory_.back().x, //no filtering at position
			trajectory_.back().y,
			trajectory_.back().time_stamp,
			vx_coef ,
			vy_coef );

    filtered_pose.cov[0] = trajectory_.back().cov_xx();
    filtered_pose.cov[1] = trajectory_.back().cov_xy();
    filtered_pose.cov[4] = trajectory_.back().cov_xy();
    filtered_pose.cov[5] = trajectory_.back().cov_yy();


	//covariance of velocity prediction, with respect to regression velocity
	Eigen::VectorXd vx_mean ( window_elements ) , vy_mean ( window_elements );
	vx_mean = R*vx_coef;
	vy_mean = R*vy_coef;
	double cov_vx = (vx - vx_mean).dot( vx - vx_mean );
	double cov_vy = (vy - vy_mean).dot( vy - vy_mean );
	double cov_vxy = (vy - vy_mean).dot( vx - vx_mean );

	//regression covariances + projected uncertainty from positions P_x / dt
	filtered_pose.cov[10] = cov_vx;// + 0.001;
	filtered_pose.cov[11] = cov_vxy;
	filtered_pose.cov[14] = filtered_pose.cov[11];
	filtered_pose.cov[15] = cov_vy;// + 0.001;

	return filtered_pose;
}


void  Cperson_bhmip::intention_precalculation()
{
	double phi , phi_prob;
	std::vector<double> s;
	phi_pose2dest_.push_back( s );
	phi_prob_.push_back( s );
	for(unsigned int i = 0; i < destinations_.size() ; ++i)
	{
		double dx = destinations_[i].x - current_pointV_.x;
		double dy = destinations_[i].y - current_pointV_.y;
		phi = diffangle( atan2(  dy , dx ) , current_pointV_.orientation());
		phi_pose2dest_.back().push_back( phi );
		//c * Pr ( phi | destination_i) calculations
		phi_prob = 100*exp( - phi*phi / phi_var_ );
		phi_prob_.back().push_back( phi_prob );
	}


}

void Cperson_bhmip::trajectory_windowing()
{
	while(  trajectory_.back().time_stamp  - trajectory_.front().time_stamp >  time_window_ )
	{
		trajectory_.pop_front();
		if(type_ == Person)//Robot and Virtual_Person do not require prediction methods
		{
			phi_pose2dest_.pop_front();
			phi_prob_.pop_front();
		}
	}
}

void Cperson_bhmip::refresh_person( double now )
{
	//this method is used when manual remove of persons is required and thus, we must
	//guarantee that the trajectory container and other variables are OK

	if (trajectory_.empty() )
	{
		while( trajectory_.back().time_stamp - now > time_window_)
		{
			trajectory_.pop_front();
			if (trajectory_.empty() ) break;
		}
	}
	now_ = now;
}

void Cperson_bhmip::prediction( double min_v_to_predict )
{
	//calculate if it is adequate to predict intentionality:
	//if target is almost stopped and decelerating
	if( current_pointV_.v() <  min_v_to_predict)
	{
		best_destination_ = Sdestination( 0, current_pointV_.x, current_pointV_.y,
				1.0, Sdestination::Stopping);
		desired_velocity_ = 0.0;//to avoid being too reactive to its environment, it remains in place v_d = 0.0
		return;
	}

	//calculate probabilities to destinations
	double norm_term(0.0),max_prob(-0.1);
	unsigned int max_prob_index;
	posterior_destinations_prob_.clear();
	for( unsigned int i = 0; i<destinations_.size() ; ++i)
	{
		posterior_destinations_prob_.push_back( destinations_[i].prob);
		for (unsigned int j = 0; j < phi_prob_.size() ; ++j)
		{
			//phi_probabilities [ time ] [ destination ]
			posterior_destinations_prob_[i] *= phi_prob_[j][i];
		}
		norm_term += posterior_destinations_prob_[i];
		if( max_prob < posterior_destinations_prob_[i])
		{
			max_prob_index = i;
			max_prob = posterior_destinations_prob_[i];
		}
	}

	//uncertain behavior of prediction depending on certain threshold
	if( 0) //max_prob < 1e16 )//hardcoded threshold corresponds aprox to a diff of 45º
	{
		best_destination_ = Sdestination( 0, current_pointV_.x, current_pointV_.y,
				1.0, Sdestination::Uncertain);
		return;
	}
	//normalization of probabilities
	norm_term = 1/norm_term;
	if ( !is_nan( norm_term ) && !destinations_.empty() )
	{
		for (unsigned int i = 0 ; i < destinations_.size() ; ++i)
			posterior_destinations_prob_[i] *= norm_term;
		best_destination_ = destinations_[max_prob_index];
		best_destination_.prob = posterior_destinations_prob_[max_prob_index];
	}
	else
	{
		best_destination_ = Sdestination( 0, current_pointV_.x, current_pointV_.y,
				1.0, Sdestination::Uncertain);
	}
}

void Cperson_bhmip::reset(  )
{
	trajectory_.clear();
	current_pointV_ = SpointV_cov();
	diff_pointV_ = SpointV_cov();
	phi_pose2dest_.clear();
	phi_prob_.clear();
	force_to_goal_ = Sforce() ;
	force_int_person_ = Sforce() ;
	force_obstacle_ = Sforce() ;
	force_int_robot_ = Sforce() ;
}


void Cperson_bhmip::rotate_and_translate_trajectory(double R, double thetaZ, double linear_vx, double linear_vy, double v_rot_x, double v_rot_y, std::vector<double> vect_odom_eigen_tf, bool debug_odometry){
	// Function for local tracking. This function change the frame of the Current_pose and the window of poses for the person,
	// tacking into account the actual position of the robot.
	// Also compensate the rotational velocities applied to the person when the robot turns.
		// R -> Robot Translation ; thetaZ -> Robot rotation


	if(debug_odometry){
	std::cout << "!!!!!!!!!!!!!!!!!! (people prediction) rotate_and_translate_trajectory !!!!!!!!!!!!!" << std::endl;
	}
	hom2_ant.clear();

	Eigen::MatrixXd homT1; // homogeneous transfor para pasar los tracks al nuevo frame de la odometria.
	Eigen::MatrixXd homT1vis;
	Eigen::MatrixXd homT2;
	Eigen::MatrixXd J; // necesaria para introducir el aumento de la covarianza por culpa del error en R=traslacion y en thetaZ=rotación.
	Eigen::MatrixXd H_cov;
	Eigen::MatrixXd P;
	homT1.resize(3, 3);
	homT1vis.resize(3, 3);
	homT2.resize(3, 3);
	J.resize(4, 2);
	H_cov.resize(4, 4);
	P.resize(2, 2);

	/*homT1.row(0) << cos(thetaZ), -sin(thetaZ), R * cos(thetaZ/2);
	homT1.row(1) << sin(thetaZ), cos(thetaZ), R * sin(thetaZ/2);
	homT1.row(2) << 0, 0, 1;*/
	
	if(debug_odometry){
	std::cout << "(people prediction) homT1 tf=" << std::endl;
	std::cout << "["<<vect_odom_eigen_tf[0]<<","<<vect_odom_eigen_tf[1]<<","<< vect_odom_eigen_tf[2]<<"]" << std::endl;
	std::cout << "["<<vect_odom_eigen_tf[3]<<","<<vect_odom_eigen_tf[4]<<","<< vect_odom_eigen_tf[5]<<"]" << std::endl;
	std::cout << "["<<vect_odom_eigen_tf[6]<<","<<vect_odom_eigen_tf[7]<<","<< vect_odom_eigen_tf[8]<<"]"  << std::endl;
	}
	
	homT1.row(0) << vect_odom_eigen_tf[0], vect_odom_eigen_tf[1], vect_odom_eigen_tf[2];
	homT1.row(1) << vect_odom_eigen_tf[3], vect_odom_eigen_tf[4], vect_odom_eigen_tf[5];
	homT1.row(2) << vect_odom_eigen_tf[6], vect_odom_eigen_tf[7], vect_odom_eigen_tf[8];

	if(debug_odometry){
	homT1vis.row(0) << cos(thetaZ), -sin(thetaZ), R * cos(thetaZ/2);
	homT1vis.row(1) << sin(thetaZ), cos(thetaZ), R * sin(thetaZ/2);
	homT1vis.row(2) << 0, 0, 1;
		std::cout << "homT1vis _ prediction  = [ " << homT1(0, 0) << " , " << homT1(0, 1)
				<< " , " << homT1(0, 2) << std::endl << "               "
				<< homT1(1, 0) << " , " << homT1(1, 1) << " , " << homT1(1, 2)
				<< std::endl << "               " << homT1(2, 0) << " , "
				<< homT1(2, 1) << " , " << homT1(2, 2) << std::endl;
	}

	//homT2 = homT1.inverse();
	homT2 = homT1;//.inverse();

	if(debug_odometry){
		std::cout << "homT2 _ prediction = [ " << homT2(0, 0) << " , " << homT2(0, 1)
				<< " , " << homT2(0, 2) << std::endl << "               "
				<< homT2(1, 0) << " , " << homT2(1, 1) << " , " << homT2(1, 2)
				<< std::endl << "               " << homT2(2, 0) << " , "
				<< homT2(2, 1) << " , " << homT2(2, 2) << std::endl;
	}
		/*hom2_ant.push_back(homT2(0, 0));
		hom2_ant.push_back(homT2(0, 1));
		hom2_ant.push_back(homT2(0, 2));
		hom2_ant.push_back(homT2(1, 0));
		hom2_ant.push_back(homT2(1, 1));
		hom2_ant.push_back(homT2(1, 2));
		hom2_ant.push_back(homT2(2, 0));
		hom2_ant.push_back(homT2(2, 1));
		hom2_ant.push_back(homT2(2, 2));
		hom2_tf.push_front(hom2_ant);*/

	H_cov.row(0) << cos(thetaZ), sin(thetaZ), 0, 0;
	H_cov.row(1) << -sin(thetaZ), cos(thetaZ), 0, 0;
	H_cov.row(2) << 0, 0, cos(thetaZ), sin(thetaZ);
	H_cov.row(3) << 0, 0, -sin(thetaZ), cos(thetaZ);

	P.row(0) << 10 * R / 100, 0; //[G_R^2       0    ]
	P.row(1) << 0, 10 * thetaZ / 100; //[  0    G_thetaZ^2]

	// Robot local velocity
	trajectory_local_velocity_x_.push_back(linear_vx);
	trajectory_local_velocity_y_.push_back(linear_vy);


	// Change tracks trajectories to the actual robot frame.

	for(unsigned int i=0; i<trajectory_.size(); i++){

		/*Eigen::MatrixXd homT2_act;
		for(unsigned int j=0;j=<i;j++){
			homT2_act=homT2_act*hom2_tf
		}*/

		SpointV_cov track_ant=trajectory_[i];
		if(debug_odometry){
			std::cout << " Before (people prediction) trajectory_("<<i<<")"<< std::endl;
				trajectory_[i].print();
		}
		SpointV_cov track_act; //trajectory point transformed.

		Eigen::MatrixXd pos_ant(3,1);
		Eigen::MatrixXd pos_act(3,1);
		Eigen::MatrixXd vel_ant(3,1);
		Eigen::MatrixXd vel_act(3,1);
		Eigen::MatrixXd cov_ant(4,4);
		Eigen::MatrixXd cov_act(4,4);
		Eigen::MatrixXd robot_linear_vel(3,1);

		robot_linear_vel(0, 0) = linear_vx;
		robot_linear_vel(1, 0) = linear_vy;
		robot_linear_vel(2, 0) = 0;

		pos_ant(0, 0) = track_ant.x;
		pos_ant(1, 0) = track_ant.y;
		pos_ant(2, 0) = 1;

		pos_act = homT2 * pos_ant;

		vel_ant.row(0) << track_ant.vx;
		vel_ant.row(1) << track_ant.vy;
		vel_ant.row(2) << 0;

		vel_act = homT2 * vel_ant;

		//vel_act=vel_act-robot_linear_vel- w_robot x pos_act;

		J.row(0) << -cos(thetaZ), -sin(thetaZ) * track_ant.x
						+ cos(thetaZ) * track_ant.y + R * sin(thetaZ );
		J.row(1) << sin(thetaZ), -cos(thetaZ) * track_ant.x
						- sin(thetaZ) * track_ant.y + R * cos(thetaZ );
		J.row(2) << 0, -sin(thetaZ) * track_ant.vx + cos(thetaZ) * track_ant.vy;
		J.row(3) << 0, -cos(thetaZ) * track_ant.vx - sin(thetaZ) * track_ant.vy;

		cov_ant.row(0) << track_ant.cov[0], track_ant.cov[1], track_ant.cov[2], track_ant.cov[3];
		cov_ant.row(1) << track_ant.cov[4], track_ant.cov[5], track_ant.cov[6], track_ant.cov[7];
		cov_ant.row(2) << track_ant.cov[8], track_ant.cov[9], track_ant.cov[10], track_ant.cov[11];
		cov_ant.row(3) << track_ant.cov[12], track_ant.cov[13], track_ant.cov[14], track_ant.cov[15];

		//cov_act = H_cov * cov_ant * H_cov.transpose() + J * P * J.transpose();
		cov_act = H_cov * cov_ant * H_cov.transpose();

		std::vector<double> vect_cov_act;
		vect_cov_act.reserve(16);
		vect_cov_act.resize(16, 0.0);
		vect_cov_act[0] = (double) cov_act(0, 0); // cov_xx
		vect_cov_act[1] = (double) cov_act(0, 1); //cov_xy
		vect_cov_act[2] = (double) cov_act(0, 2); // cov_xvx
		vect_cov_act[3] = (double) cov_act(0, 3); //cov_xvy
		vect_cov_act[4] = (double) cov_act(1, 0); //cov_yx
		vect_cov_act[5] = (double) cov_act(1, 1); //cov_yy
		vect_cov_act[6] = (double) cov_act(1, 2); //cov_yvx
		vect_cov_act[7] = (double) cov_act(1, 3); //cov_yvy
		vect_cov_act[8] = (double) cov_act(2, 0); // cov_vxvx
		vect_cov_act[9] = (double) cov_act(2, 1); //cov_vxvy
		vect_cov_act[10] = (double) cov_act(2, 2); // cov_vxx
		vect_cov_act[11] = (double) cov_act(2, 3); //cov_vxy
		vect_cov_act[12] = (double) cov_act(3, 0); //cov_vyvx
		vect_cov_act[13] = (double) cov_act(3, 1); //cov_vyvy
		vect_cov_act[14] = (double) cov_act(3, 2); //cov_vyx
		vect_cov_act[15] = (double) cov_act(3, 3); //cov_vyy

		track_act = SpointV_cov(pos_act(0, 0), pos_act(1, 0),track_ant.time_stamp, vel_act(0, 0), vel_act(1, 0),vect_cov_act);

		trajectory_[i]=track_act;

		if(debug_odometry){
		std::cout << "(people prediction) track_ant("<<i<<")" << std::endl;
		track_ant.print();
		std::cout << " (people prediction) track_act("<<i<<")"<< std::endl;
		track_act.print();
		std::cout << " After (people prediction) trajectory_("<<i<<")"<< std::endl;
		trajectory_[i].print();
		}
	}

	// change current_pointV_ with odometry
	SpointV_cov ant_current_pointV=current_pointV_;
	if(debug_odometry){
	std::cout << " ANT current_pointV_="<< std::endl;
	current_pointV_.print();
	}
	//SpointV_cov act_current_pointV;
	Eigen::MatrixXd pos_ant_current_pointV(3,1);
	Eigen::MatrixXd pos_act_current_pointV(3,1);
	Eigen::MatrixXd vel_ant_current_pointV(3,1);
	Eigen::MatrixXd vel_act_current_pointV(3,1);
	Eigen::MatrixXd cov_ant_current_pointV(4,4);
	Eigen::MatrixXd cov_act_current_pointV(4,4);

	pos_ant_current_pointV(0, 0) = ant_current_pointV.x;
	pos_ant_current_pointV(1, 0) = ant_current_pointV.y;
	pos_ant_current_pointV(2, 0) = 1;

	pos_act_current_pointV = homT2 * pos_ant_current_pointV;

	vel_ant_current_pointV.row(0) << ant_current_pointV.vx;
	vel_ant_current_pointV.row(1) << ant_current_pointV.vy;
	vel_ant_current_pointV.row(2) << 0;

	vel_act_current_pointV = homT2 * vel_ant_current_pointV;

	J.row(0) << -cos(thetaZ), -sin(thetaZ) * ant_current_pointV.x + cos(thetaZ) * ant_current_pointV.y + R * sin(thetaZ / 2);
	J.row(1) << sin(thetaZ), -cos(thetaZ) * ant_current_pointV.x - sin(thetaZ) * ant_current_pointV.y + R * cos(thetaZ / 2);
	J.row(2) << 0, -sin(thetaZ) * ant_current_pointV.vx + cos(thetaZ) * ant_current_pointV.vy;
	J.row(3) << 0, -cos(thetaZ) * ant_current_pointV.vx - sin(thetaZ) * ant_current_pointV.vy;

	cov_ant_current_pointV.row(0) << ant_current_pointV.cov[0], ant_current_pointV.cov[1], ant_current_pointV.cov[2], ant_current_pointV.cov[3];
	cov_ant_current_pointV.row(1) << ant_current_pointV.cov[4], ant_current_pointV.cov[5], ant_current_pointV.cov[6], ant_current_pointV.cov[7];
	cov_ant_current_pointV.row(2) << ant_current_pointV.cov[8], ant_current_pointV.cov[9], ant_current_pointV.cov[10], ant_current_pointV.cov[11];
	cov_ant_current_pointV.row(3) << ant_current_pointV.cov[12], ant_current_pointV.cov[13], ant_current_pointV.cov[14], ant_current_pointV.cov[15];

	cov_act_current_pointV = H_cov * cov_ant_current_pointV * H_cov.transpose() + J * P * J.transpose();

	std::vector<double> vect_cov_act_current_pointV;
	vect_cov_act_current_pointV.reserve(16);
	vect_cov_act_current_pointV.resize(16, 0.0);
	vect_cov_act_current_pointV[0] = (double) cov_act_current_pointV(0, 0); // cov_xx
	vect_cov_act_current_pointV[1] = (double) cov_act_current_pointV(0, 1); //cov_xy
	vect_cov_act_current_pointV[2] = (double) cov_act_current_pointV(0, 2); // cov_xvx
	vect_cov_act_current_pointV[3] = (double) cov_act_current_pointV(0, 3); //cov_xvy
	vect_cov_act_current_pointV[4] = (double) cov_act_current_pointV(1, 0); //cov_yx
	vect_cov_act_current_pointV[5] = (double) cov_act_current_pointV(1, 1); //cov_yy
	vect_cov_act_current_pointV[6] = (double) cov_act_current_pointV(1, 2); //cov_yvx
	vect_cov_act_current_pointV[7] = (double) cov_act_current_pointV(1, 3); //cov_yvy
	vect_cov_act_current_pointV[8] = (double) cov_act_current_pointV(2, 0); // cov_vxvx
	vect_cov_act_current_pointV[9] = (double) cov_act_current_pointV(2, 1); //cov_vxvy
	vect_cov_act_current_pointV[10] = (double) cov_act_current_pointV(2, 2); // cov_vxx
	vect_cov_act_current_pointV[11] = (double) cov_act_current_pointV(2, 3); //cov_vxy
	vect_cov_act_current_pointV[12] = (double) cov_act_current_pointV(3, 0); //cov_vyvx
	vect_cov_act_current_pointV[13] = (double) cov_act_current_pointV(3, 1); //cov_vyvy
	vect_cov_act_current_pointV[14] = (double) cov_act_current_pointV(3, 2); //cov_vyx
	vect_cov_act_current_pointV[15] = (double) cov_act_current_pointV(3, 3); //cov_vyy

	//double track_vx=vel_act_current_pointV(0,0) - v_rot_x;
	//double track_vy=vel_act_current_pointV(1,0) - v_rot_y;

	//vel_act_current_pointV(0,0)=track_vx;
	//vel_act_current_pointV(1,0)=track_vy;



	current_pointV_ = SpointV_cov(pos_act_current_pointV(0, 0), pos_act_current_pointV(1, 0), ant_current_pointV.time_stamp, vel_act_current_pointV(0,0), vel_act_current_pointV(1,0),vect_cov_act_current_pointV);
	if(debug_odometry){
	std::cout << " AcT current_pointV_="<< std::endl;
	current_pointV_.print();
	}
}
