#ifndef iri_geometry_H
#define iri_geometry_H

#include <vector>
const static double PI = 3.1415926535897932384626433;

/**
 *	\brief iri_geometry library
 *
 *	It consists of a set of data structures, similar to structs, where all values are accessible from
 *	the outside and a set of simple methods for each geometry primitive.
 *
 *
 *  Note: In general, these classes are designed as simple data structures with methods associated
 *  By no means they are intended to be created as pointers and new or used polymorphically.
 *  No dynamical binding of the functions
 *  has been designed (only statical, beware on how you declare them), so they won't work!!!!
 *  Simply use it as data types, as can be seen in test/test_geometry.
 *
  */


/**
 *	\brief Sforce Struct
 *
 *	data struct describing a 2d force (fx,fy)
 *	and their corresponding methods
 *	TODO: add force covariances
 */
class Spoint;
class Sforce
{
  public:
	double fx, fy;
	Sforce();
	Sforce(double fx_ , double fy_);
	double module( void );
	double module( double r2 );//r2 if the square factor to compate fx and r2*fy
	double module2( void );
	double module2( double r2 );
	Sforce operator+ (Sforce f2) const;
	void sum(Sforce f2);//a method similar to +=
	Sforce& operator+= (Sforce f2);
	Sforce operator* (double k) const;
	double operator* (Spoint dr) const;
    Sforce operator- (Sforce f2) const;
	void print() const;
};

/**
 *	\brief Spoint Struct
 *
 *	data struct describing a 2d point (x,y).
 */
class Spoint
{
  public:
	double x;
	double y;
	double time_stamp; //sec ;
	Spoint();
	Spoint( double x_ , double y_ , double time_stamp_=0.0) ;
	Spoint operator+ (Spoint p2) const ;
	Spoint operator- (Spoint p2) const ;
	Spoint operator* (double k) const ;
	double distance(Spoint p2 = Spoint() ) const;
	double distance2( Spoint p2 = Spoint() ) const;
	Spoint propagate( double dt , Sforce f = Sforce(), double desired_velocity=2.0) const;
	void print() const;
};

/**
 *	\brief Spoint_cov Struct
 *
 *	data struct describing a 2d point (x,y). and its covariance. This covariance is represented as
 *	a vector of 4 elements
 */
class Spoint_cov : public Spoint
{
  public:
	std::vector<double> cov;
	Spoint_cov();
	Spoint_cov( double x_ , double y_ , double time_stamp_=0.0, const std::vector<double>& cov_= std::vector<double>() );
	double cov_dist( Spoint p2=Spoint() ) const;
    Spoint_cov operator+ (Spoint_cov p2) const ;
    Spoint_cov operator- (Spoint_cov p2) const ;
    Spoint_cov operator* (double k) const ;
    double cov_xx() const;
    double cov_yy() const;
    double cov_xy() const;
	Spoint_cov propagate( double dt , Sforce f = Sforce(), double desired_velocity=2.0) const;
	void print() const;
};


/**
 *	\brief SpointV Struct
 *
 *	data struct describing a 2d point (x,y) and its velocities (derivate)
 */
class SpointV : public Spoint
{
  public:
	double vx;
	double vy;
	SpointV();
	SpointV( double x_ , double y_, double time_stamp_=0.0, double vx_=0.0, double vy_=0.0 ) ;
	SpointV operator+ (SpointV p2) const ;
	SpointV operator- (SpointV p2) const ;
	SpointV operator* (double k) const ;
	SpointV propagate( double dt , Sforce f = Sforce(), double desired_velocity=2.0) const;
	double orientation() const;
	double v() const;
	void norm_v(double v);
	double angle_heading_point( Spoint p2 ) const;
	void print() const;
};

/**
 *	\brief SpointV Struct
 *
 *	data struct describing a 2d point (x,y) and its velocities (derivate)
 */
class SpointV_cov : public SpointV
{
  public:
	std::vector<double> cov;
	SpointV_cov();
	SpointV_cov(double x_ , double y_, double time_stamp_=0.0, double vx_=0.0, double vy_=0.0,
			const std::vector<double>& cov_= std::vector<double>());
	SpointV_cov(SpointV p,	const std::vector<double>& cov_= std::vector<double>());
	SpointV_cov(Spoint_cov p );
	double cov_dist( Spoint p2=Spoint() ) const;
	double cov_dist( Spoint p2 , double &det) const;
	double cov_distV( SpointV_cov p2, double &distV, double &distxv  ) const;
    SpointV_cov operator+ (SpointV_cov p2) const ;
    SpointV_cov operator- (SpointV_cov p2) const ;
    SpointV_cov operator* (double k) const ;
    SpointV_cov propagate( double dt , Sforce f = Sforce(), double desired_velocity=2.0 ) const;
    double cov_xx() const;
    double cov_yy() const;
    double cov_xy() const;
    Spoint_cov toSpoint_cov() const;
    void print() const;
};
/**
 *	\brief Spose Struct
 *
 *	data struct describing a pose consisting of position, velocity
 *	and time.
 */
class Spose : public Spoint
{
  public:
	double theta;
	double v;
	double w;
	Spose();
	Spose( double x_ , double y_ , double time_stamp_=0.0 , double theta_=0.0 ,  double v_=0.0, double w_=0.0);
	Spose operator+ (Spose p2) const;
	Spose operator- (Spose p2) const;
	Spose operator* (double k) const;
	double distance(Spoint p2) const;
	double angle_heading_pose( Spose p2 ) const;
	double social_distance( Spose p2 ) const;
	void print() const;
};
/**
 *	\brief SposeCov Struct
 *
 *	data struct describing a pose and its covariance (only in position)
 *	It also supports distance to other pose or distances-based on covariances
 */
class Spose_cov : public Spose
{
  public:
	std::vector<double> cov;
	Spose_cov();
	Spose_cov( double x_ , double y_ ,
			double time_stamp_=0.0 ,double theta_=0.0 ,  double v_=0.0, double w_=0.0,
			const std::vector<double>& cov_= std::vector<double>());
	Spose_cov( Spose pose_, const std::vector<double>& cov_= std::vector<double>());
	Spose_cov( SpointV_cov point );//conversion from point to pose
	double distance( Spose_cov p2 ) const;
	double cov_dist( Spose p2 ) const;
	double cov_dist( Spose_cov p2 ) const;
};

/**
 *	\brief Sdestination Struct
 *
 *	data struct describing the destination information:
 *	position, prior probability and type{fixed, temporal}
 */
class Sdestination : public Spoint
{
  public:
	enum destination_type { Map_goal=0 , Stopping, Uncertain};
	Sdestination( int id_=0, double x_=0.0 , double y_=0.0 ,
			double prob_=0.0 ,
			destination_type type_=Map_goal ,
			std::vector<int> neighbours_ids_ = std::vector<int>());
	~Sdestination();
	int id;
	double prob;
	destination_type type;
	std::vector<int> neighbours_ids;
	bool operator== (Sdestination d2) const;
	void print() const;
};

/**
 *
 *  \brief detectionObservation Struct to be deprecated... pass a vector of Spoint/Pose and
 *  a vector of id's TODO
 *
 */

class SdetectionObservation : public SpointV_cov
{
  public:
	int id;
	SdetectionObservation();
	SdetectionObservation( int id_ , double time_stamp_=0.0, double x_=0.0, double y_=0.0, double vx_=0.0, double vy_=0.0,
			const std::vector<double>& cov_= std::vector<double>());
	SdetectionObservation( int id_ , SpointV_cov pointV_cov_);
	void print() const;
};



double diffangle(double alpha , double beta);
inline bool is_nan(double x) { return x != x;}

#endif
