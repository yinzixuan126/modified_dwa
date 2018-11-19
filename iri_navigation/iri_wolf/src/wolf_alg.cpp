#include "wolf_alg.h"

WolfAlgorithm::WolfAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

WolfAlgorithm::~WolfAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void WolfAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();
  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// WolfAlgorithm Public API
