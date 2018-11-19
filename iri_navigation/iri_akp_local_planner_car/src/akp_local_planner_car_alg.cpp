#include "akp_local_planner_car_alg.h"

AkpLocalPlannerAlgorithm::AkpLocalPlannerAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

AkpLocalPlannerAlgorithm::~AkpLocalPlannerAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void AkpLocalPlannerAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// AkpLocalPlannerAlgorithm Public API
