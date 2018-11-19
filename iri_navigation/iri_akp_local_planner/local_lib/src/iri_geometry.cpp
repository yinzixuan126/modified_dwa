#include "iri_geometry.h"
#include <math.h>
#include <iostream>
#include <Eigen/Dense>

// Spoint functions ------------------------------------------
Spoint::Spoint() : x(0) , y(0),  time_stamp(0) { }

Spoint::Spoint( double x_ , double y_ , double time_stamp_) :
		x(x_), y(y_), time_stamp(time_stamp_) { }
Spoint Spoint::operator+ (Spoint p2) const
{
	return Spoint(  x + p2.x , y + p2.y , time_stamp );
}
Spoint Spoint::operator- (Spoint p2) const
{
	return Spoint(  x - p2.x , y - p2.y , time_stamp);
}
Spoint Spoint::operator* (double k) const
{
	return Spoint(  x *k , y *k , time_stamp);
}
double Spoint::distance(Spoint p2) const
{
	return sqrt( distance2(p2)  );
}
double Spoint::distance2(Spoint p2) const
{
	return (x-p2.x)*(x-p2.x) + (y-p2.y)*(y-p2.y);
}
Spoint Spoint::propagate( double dt , Sforce f, double desired_velocity) const
{
	return *this;
}
void Spoint::print() const
{
	std::cout << "(x , y ,t) = (" << x << " , " << y  << " | " << time_stamp << " )" << std::endl;
}

//Spoint_cov methods -----------------------------------------------------------
Spoint_cov::Spoint_cov():
		Spoint()
{
	cov.reserve(4);
	cov.resize(4,0.0);
	cov[0] = 1.0;
	cov[3] = 1.0;
}
Spoint_cov::Spoint_cov( double x_ , double y_ , double time_stamp_,
		const std::vector<double>& cov_ ):
		Spoint( x_,y_,time_stamp_ )
{
	if ( cov_.empty() || cov_.size() < (std::size_t) 4 )
	{
		cov.reserve(4);
		cov.resize(4,0.0);
		cov[0] = 1.0;
		cov[3] = 1.0;
	}
	else
	{
		cov = cov_;
	}
}
double Spoint_cov::cov_dist( Spoint p2 ) const
{
	//analitical inversion of the 2x2 matrix
	//       det =   cov_xx * cov_yy - cov_xy * cov_xy
	double cov_det = cov[0] * cov[3] - cov[1] * cov[2];
	double dx = x - p2.x;
	double dy = y - p2.y;
	return ( dx*dx*cov[3] - 2*dx*dy*cov[1] + dy*dy*cov[0] )/cov_det;
}
Spoint_cov Spoint_cov::operator+ (Spoint_cov p2) const
{
    return Spoint_cov(  x + p2.x , y + p2.y , time_stamp , cov);
}
Spoint_cov Spoint_cov::operator- (Spoint_cov p2) const
{
    return Spoint_cov(  x - p2.x , y - p2.y , time_stamp , cov);
}
Spoint_cov Spoint_cov::operator* (double k) const
{
    return Spoint_cov(  x*k , y*k , time_stamp , cov);
}
double Spoint_cov::cov_xx() const
{
	return cov[0];
}
double Spoint_cov::cov_yy() const
{
	return cov[3];
}
double Spoint_cov::cov_xy() const
{
	return cov[1];
}
void Spoint_cov::print() const
{
    Spoint::print();
    std::cout << "cov = [" << cov[0] << " , " << cov[1] << std::endl;
    std::cout << "       " << cov[2] << " , " << cov[3] << " ]" << std::endl;
}

//SpointV methods -----------------------------------------------------------
SpointV::SpointV() : Spoint(), vx(0), vy(0) { }

SpointV::SpointV( double x_ , double y_, double time_stamp_, double vx_, double vy_ ) :
		Spoint(x_,y_,time_stamp_), vx(vx_),vy(vy_) { }
SpointV SpointV::operator+ (SpointV p2) const
{
	return SpointV(  x + p2.x , y + p2.y , time_stamp,  vx + p2.vx , vy + p2.vy );
}
SpointV SpointV::operator- (SpointV p2) const
{
	return SpointV(  x - p2.x , y - p2.y , time_stamp-p2.time_stamp,  vx - p2.vx , vy - p2.vy );
}
SpointV SpointV::operator* ( double k) const
{
	return SpointV(  x * k , y * k, time_stamp,  vx * k , vy * k );
}
void SpointV::print() const
{
	std::cout << "(x , y ,t) = (" << x << " , " << y  << " | " << time_stamp << " )" <<
			"     (vx,vy) = ( " << vx << " , " << vy << " )" << std::endl;
}
SpointV SpointV::propagate( double dt , Sforce f, double desired_velocity) const
{
	//linear propagation (uniformly accelerated system)
	double vxn = vx + f.fx*dt;
	double vyn = vy + f.fy*dt;
	double dx = vx*dt + f.fx*dt*dt*0.5;
	double dy = vy*dt + f.fy*dt*dt*0.5;

	//apply constrains:
	double v = sqrt( vxn*vxn + vyn*vyn );
	if ( v > desired_velocity)
	{
		vxn *= desired_velocity /v;
		vyn *= desired_velocity /v;
		dx = vxn*dt;
		dy = vyn*dt;
	}

	return SpointV( x+dx,y+dy, time_stamp+dt, vxn, vyn);
}
double SpointV::orientation() const
{
	return atan2( vy, vx );
}

double SpointV::v() const
{
	return sqrt( vy*vy + vx*vx );
}

void SpointV::norm_v(double v)
{
	double theta = this->orientation();
	vx = cos(theta)*v;
	vy = sin(theta)*v;
}

double SpointV::angle_heading_point( Spoint p2 ) const
{
	double dx = p2.x - x;
	double dy = p2.y - y;
	return diffangle( atan2( dy , dx ), orientation() );
}

//SpointV_cov methods -----------------------------------------------------------
SpointV_cov::SpointV_cov() :
		SpointV()
{
	cov.reserve(16);
	cov.resize(16,0.0);
	cov[0] = 0.4;
	cov[5] = 0.4;
	cov[10] = 0.1;
	cov[15] = 0.1;
}
SpointV_cov::SpointV_cov(double x_ , double y_, double time_stamp_, double vx_, double vy_,
		const std::vector<double>& cov_) :
		SpointV(x_,y_,time_stamp_,vx_,vy_)
{
	if ( cov_.empty() || cov_.size() < (std::size_t) 16 )
	{
		cov.reserve(16);
		cov.resize(16,0.0);
		cov[0] = 0.4;
		cov[5] = 0.4;
		cov[10] = 0.1;
		cov[15] = 0.1;
	}
	else
	{
		cov = cov_;
	}
}
SpointV_cov::SpointV_cov(SpointV p, const std::vector<double>& cov_) :
		SpointV( p.x , p.y, p.time_stamp, p.vx, p.vy)
{
	if ( cov_.empty() || cov_.size() < (std::size_t) 16 )
	{
		cov.reserve(16);
		cov.resize(16,0.0);
		cov[0] = 0.4;
		cov[5] = 0.4;
		cov[10] = 0.1;
		cov[15] = 0.1;
	}
	else
	{
		cov = cov_;
	}
}
SpointV_cov::SpointV_cov(Spoint_cov p ):
		SpointV( p.x , p.y, p.time_stamp, 0.0, 0.0)
{
	cov.reserve(16);
	cov.resize(16,0.0);
	cov[0] = p.cov[0];
	cov[1] = p.cov[1];
	cov[4] = p.cov[2];
	cov[5] = p.cov[3];
	cov[10] = 0.1;
	cov[15] = 0.1;
}

double SpointV_cov::cov_dist( Spoint p2 ) const
{
	// TODO: consider velocities, right now is a covariance distance of positions
	//       det =   cov_xx * cov_yy - cov_xy * cov_xy
	double cov_det = cov[0] * cov[5] - cov[1] * cov[4];
	double dx = x - p2.x;
	double dy = y - p2.y;
	//
	return ( dx*dx*cov[5] - 2*dx*dy*cov[1] + dy*dy*cov[0] )/cov_det;
}
double SpointV_cov::cov_dist( Spoint p2 , double &det) const
{
	det = cov[0] * cov[5] - cov[1] * cov[4];
	double dx = x - p2.x;
	double dy = y - p2.y;
	return ( dx*dx*cov[5] - 2*dx*dy*cov[1] + dy*dy*cov[0] )/det;
}

double SpointV_cov::cov_distV( SpointV_cov p2, double &distV, double &distxv ) const
{
	// TODO: consider velocities, right now is a covariance distance of positions
	//       det =   cov_xx * cov_yy - cov_xy * cov_xy
	// SpointV_cov external for actual track, SpointV_cov internal for the initial Spoint_cov for this track in the first time that the cross is made.
	double cov_det = cov[0] * cov[4] - cov[1] * cov[5];
	double dx = x - p2.x;
	double dy = y - p2.y;
	double distx=( dx*dx*cov[5] - 2*dx*dy*cov[1] + dy*dy*cov[0] )/cov_det;

	double cov_detV = cov[10] * cov[14] - cov[11] * cov[15];
	double dvx = vx - p2.vx;
	double dvy = vy - p2.vy;
	distV=( dvx*dvx*cov[15] - 2*dvx*dvy*cov[11] + dvy*dvy*cov[10] )/cov_detV;

	distxv=distx+distV;

	return distx;

}

SpointV_cov SpointV_cov::operator+ (SpointV_cov p2) const
{
    return SpointV_cov(  x + p2.x , y + p2.y , time_stamp , vx + p2.vx , vy + p2.vy ,cov);
}
SpointV_cov SpointV_cov::operator- (SpointV_cov p2) const
{
    return SpointV_cov(  x - p2.x , y - p2.y , time_stamp-p2.time_stamp,  vx - p2.vx , vy - p2.vy ,cov);
}
SpointV_cov SpointV_cov::operator* (double k) const
{
    return SpointV_cov(  x * k , y * k, time_stamp,  vx * k , vy * k , cov);
}
SpointV_cov SpointV_cov::propagate( double dt , Sforce f, double desired_velocity) const
{
	SpointV new_point = SpointV::propagate(dt,f,desired_velocity);
	Eigen::MatrixXd cov_prev(4 , 4 );
	for (unsigned int i = 0; i < 4; ++i)
	{
		cov_prev.row(i) << cov[i*4],cov[i*4+1],cov[i*4+2],cov[i*4+3];
	}
	//std::cout << "cov_ prev = \n" <<  cov_prev << std::endl;
	Eigen::MatrixXd Phi(4 , 4 );
	Phi = Eigen::MatrixXd::Identity(4,4);
	Phi(0,2) = dt;
	Phi(1,3) = dt;
	//std::cout << "Phi = " << Phi << std::endl;
	Eigen::MatrixXd G(4 , 2 );
	G = Eigen::MatrixXd::Zero(4,2);
	G(0,0) = dt*dt/2;
	G(1,1) = dt*dt/2;
	G(2,0) = dt;
	G(3,1) = dt;
	//std::cout << "G = " << G << std::endl;
	Eigen::MatrixXd new_cov(4,4);
	Eigen::MatrixXd cov_f(2,2);
	cov_f = Eigen::MatrixXd::Zero(2,2);
	cov_f(0,0) = 0.3;//TODO this covariance should be associated to the corresponding force and not generic
	cov_f(1,1) = 0.3;
	//std::cout << "f = " << cov_f << std::endl;
	new_cov = Phi*cov_prev*Phi.transpose() + G*cov_f*G.transpose();
	std::vector<double> new_cov_std(16,0.0);
	for(unsigned int i = 0; i<16; ++i)
	{
		new_cov_std[i] = new_cov( i/4 , i%4 );
	}
	//std::cout << "new cov = \n" <<  new_cov << std::endl;
	return SpointV_cov( new_point, new_cov_std );
}
double SpointV_cov::cov_xx() const
{
	return cov[0];
}
double SpointV_cov::cov_yy() const
{
	return cov[5];
}
double SpointV_cov::cov_xy() const
{
	return cov[1];
}
Spoint_cov SpointV_cov::toSpoint_cov() const
{
	std::vector<double> cov_point;
	cov_point.reserve(4);
	cov_point.resize(4,0.0);
	cov_point[0] = cov[0];
	cov_point[1] = cov[1];
	cov_point[2] = cov[4];
	cov_point[3] = cov[5];
	return Spoint_cov( x,y,time_stamp, cov_point );
}
void SpointV_cov::print() const
{
    SpointV::print();
    std::cout << "covariance = [ " << cov[0] << " , " << cov[1] << " , " <<cov[2]  << " , " << cov[3] << std::endl <<
    		     "               " << cov[4] << " , " << cov[5] << " , " <<cov[6]  << " , " << cov[7] << std::endl <<
    		     "               " << cov[8] << " , " << cov[9] << " , " <<cov[10]  << " , " << cov[11] << std::endl <<
    		     "               " << cov[12] << " , " << cov[13] << " , " <<cov[14]  << " , " << cov[15] << std::endl;
}

//Spose methods -----------------------------------------------------------
Spose::Spose() : Spoint(), theta(0)  , v(0) , w(0)  { }
Spose::Spose( double x_ , double y_ , double time_stamp_ , double theta_ , double v_, double w_) :
	Spoint(x_,y_, time_stamp_), theta(theta_), v(v_), w(w_) { }
Spose Spose::operator+ (Spose p2) const
{
	return Spose(  x + p2.x , y + p2.y , theta + p2.theta, p2.time_stamp , p2.v , p2.w );
}
Spose Spose::operator- (Spose p2) const
{
	return Spose(  x - p2.x , y - p2.y , theta - p2.theta, p2.time_stamp , p2.v , p2.w );
}
Spose Spose::operator* ( double k) const
{
	return Spose(  x * k , y *k , theta, time_stamp , v , w );
}
double Spose::distance(Spoint p2) const
{
	return sqrt(  (x-p2.x)*(x-p2.x) + (y-p2.y)*(y-p2.y) );
}
double Spose::angle_heading_pose( Spose p2 ) const
{
	double dx = p2.x - x;
	double dy = p2.y - y;
	return diffangle( atan2( dy , dx ), theta );
}
double Spose::social_distance( Spose p2 ) const
{
	double lambda = 0.0;
	double phi = this->angle_heading_pose( p2 );
	double anisotropy = (lambda + (1-lambda)*(1 + cos(phi))/2 );
	return this->distance(p2)/anisotropy;
}
void Spose::print() const
{
	std::cout << "(x , y | theta) = (" << x << " , " << y  << " | " << theta <<
			" )  --  at t = " << time_stamp << std::endl <<
			"(v , w) = (" << v  << " , " << w << " )" << std::endl;
}

//Spose_cov methods -----------------------------------------------------------
Spose_cov::Spose_cov():	 Spose()
{
	cov.reserve(25);
	cov.resize(25,0.0);
	cov[0] = 0.5;//xx
	cov[6] = 0.5;//yy
	cov[12] = 0.05;//theta_theta
	cov[18] = 0.1;//cov_vv
	cov[24] = 0.1;//ww
}
Spose_cov::Spose_cov(double x_ , double y_ ,
		double time_stamp_ , double theta_ ,  double v_, double w_,
		const std::vector<double>& cov_ ):
		Spose( x_,y_,time_stamp_, theta_,v_,w_ )
{
	if ( cov_.empty() || cov_.size() < (std::size_t) 25 )
	{
		cov.reserve(25);
		cov.resize(25,0.0);
		cov[0] = 0.5;//xx
		cov[6] = 0.5;//yy
		cov[12] = 0.05;//theta_theta
		cov[18] = 0.1;//cov_vv
		cov[24] = 0.1;//ww
	}
	else
	{
		cov = cov_;
	}
}
Spose_cov::Spose_cov( Spose pose_, const std::vector<double>& cov_ ):
		Spose(pose_)
{
	if ( cov_.empty() || cov_.size() < (std::size_t) 25 )
	{
		cov.reserve(25);
		cov.resize(25,0.0);
		cov[0] = 0.5;//xx
		cov[6] = 0.5;//yy
		cov[12] = 0.05;//theta_theta
		cov[18] = 0.1;//cov_vv
		cov[24] = 0.1;//ww
	}
	else
	{
		cov = cov_;
	}
}
Spose_cov::Spose_cov( SpointV_cov point ):
		Spose(point.x,point.y,point.time_stamp,point.orientation(),point.v())
{
	cov.reserve(25);
	cov.resize(25,0.0);
	cov[0] = point.cov[0];//xx
	cov[6] = point.cov[5];//yy
	cov[12] = 0.05;//theta_theta
	cov[18] = 0.1;//cov_vv
	cov[24] = 0.1;//ww
}
double Spose_cov::distance( Spose_cov p2) const
{
	return Spose::distance( Spose(p2.x,p2.y) );
}
double Spose_cov::cov_dist( Spose p2 ) const
{
	//analitical inversion of the 2x2 matrix
	double cov_xx = cov[0];
	double cov_yy = cov[6];
	double cov_xy = cov[5];
	double cov_det = cov_xx * cov_yy - cov_xy * cov_xy;
	double dx = x - p2.x;
	double dy = y - p2.y;
	return ( dx*dx*cov_yy - 2*dx*dy*cov_xy + dy*dy*cov_xx )/cov_det;
}
double Spose_cov::cov_dist( Spose_cov p2 ) const
{
	return this->cov_dist( Spose(p2.x,p2.y) );
}

//Sdestinations methods -----------------------------------------------------------
Sdestination::Sdestination( int id_, double x_ , double y_ , double prob_ ,
		Sdestination::destination_type type_, std::vector<int> neighbours_ids_ ) :
	Spoint(x_,y_), id(id_), prob(prob_) ,
	type(type_), neighbours_ids(neighbours_ids_)  { }
Sdestination::~Sdestination(){}
bool Sdestination::operator== (Sdestination d2) const
{
	if ( x == d2.x && y == d2.y )
		return true;
	else
		return false;
}

void Sdestination::print() const
{
	std::cout << "destination " << id << "at (" << x << " , " << y  <<  ")  of type " << type  <<
		"   and probability = " << prob  << "neighbours " << neighbours_ids.size() << std::endl;
}


SdetectionObservation::SdetectionObservation() :
				SpointV_cov(), id(0) { }
SdetectionObservation::SdetectionObservation( int id_ , double time_stamp_ ,
		double x_, double y_, double vx_, double vy_, const std::vector<double>& cov_) :
			SpointV_cov(x_,y_,time_stamp_,vx_,vy_,cov_) , id(id_) { }
SdetectionObservation::SdetectionObservation( int id_ , SpointV_cov pointV_cov_) :
	SpointV_cov( pointV_cov_ ), id(id_) {}
void SdetectionObservation::print() const
{
	std::cout <<  "detection Observation = " << id << " th "  << std::endl;
	SpointV_cov::print();
}


//Sforce methods
Sforce::Sforce() : fx(0.0) , fy(0.0) {}
Sforce::Sforce(double fx_ , double fy_) : fx(fx_) , fy(fy_) {}
double Sforce::module() { return sqrt(fx*fx + fy*fy);}
double Sforce::module(double r2) { return sqrt(fx*fx + r2*fy*fy);}
double Sforce::module2( ) { return fx*fx + fy*fy;}
double Sforce::module2( double r2) { return fx*fx + r2*fy*fy;}
void Sforce::print() const
{
	std::cout <<  "Force = (" << fx << " , " << fy  << " )" << std::endl;
}
Sforce Sforce::operator+ (Sforce f2) const
{
	return Sforce(  fx + f2.fx , fy + f2.fy );
}
void Sforce::sum(Sforce f2)
//a method similar to +=
{
	fx += f2.fx;
	fy += f2.fy;
}
Sforce& Sforce::operator +=(Sforce f2)
{
	fx += f2.fx;
	fy += f2.fy;
	return *this;
}

Sforce Sforce::operator* (double k) const
{
	return Sforce(  fx *k , fy *k );
}

double Sforce::operator* (Spoint dr) const
{
	return this->fx * dr.x + this->fy * dr.y;
}
Sforce Sforce::operator- (Sforce f2) const
{
    return Sforce(  fx - f2.fx , fy - f2.fy );
}

double diffangle(double alpha , double beta)
{
	  double delta = alpha - beta;
	  if ( alpha >= beta )
	  {
	    while (delta >  PI)
	      delta -= 2*PI;
	  }
	  else
	  {
		  while (delta < -PI )
			  delta += 2*PI;
	  }
	  return delta;
}
