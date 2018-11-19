#include "rand_gmm.h"
#include <stdlib.h>
#include <math.h>
#include <time.h>

double rand_uniform (  )
{
	return (double)rand()/(double)RAND_MAX;
}

double rand_uniform ( double min, double max )
{
	return min + (max-min)* rand_uniform();
}

int rand_uniform_discrete ( int min, int max )
{
	//return min + int ( (double)(max-min+1)* rand_uniform());
	return min + rand() % ( max-min+1 );
}

double rand_normal(double mu,double sigma)
{
	double u = rand_uniform();
	double v = rand_uniform();
	double x = sqrt( -2.0*log(u) )*cos( 2*M_PI*v );
	return mu + sigma*x;
}

double rand_gmm ( std::vector<Sgauss> & gmm)
{
		double p = rand_uniform () , p_accumulated = 0.0;
		unsigned int i = 0;
		while ( i < gmm.size() && p_accumulated < p)
		{
			p_accumulated += gmm.at(i).w;
			i++;
		}
		i--;
		return rand_normal ( gmm.at(i).mu , gmm.at(i).sigma );
}

void rand_seed()
{
	srand ( time(NULL) );
}
