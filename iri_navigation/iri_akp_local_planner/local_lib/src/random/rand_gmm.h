//library to be deprecated... just use c++11

#ifndef rand_gmm_h
#define rand_gmm_h



#include <vector>

struct Sgauss
{
	double mu, sigma, w;
	Sgauss ( ) : mu(0.0) , sigma(0.0) , w(0.0) { }
	Sgauss ( double mu_ , double sigma_ , double w_ ) :	mu(mu_) , sigma(sigma_) , w(w_)	{ }
};

/**
 *
 * \brief Randon number generator: uniform distribution 
 *
 * [0,1]
 * 
*/
double rand_uniform (  );
/**
 *
 * \brief Randon number generator: uniform distribution
 *
 * [min,max]
 *
*/
double rand_uniform ( double min, double max );

/**
 *
 * \brief Randon number generator: uniform discrete distribution
 *
 * {min,...,max}
 *
*/
int rand_uniform_discrete ( int min, int max );

/**
 *
 * \brief Randon number generator : normal distribution
 *
 * Implementation of the Muller-Box method
 * 
*/
double rand_normal (double mu,double sigma);

/**
 *
 * \brief Randon number generator : gaussian mixture model distribution
 *
 * N weighted gaussian sources. Strong condition: sum( w_n ) = 1
 * 
*/
double rand_gmm ( std::vector<Sgauss> & gmm);

/**
 *
 * \brief Random seed
 *
 *
*/
void rand_seed();

#endif
