#include "fake_laser_gen_alg.h"

FakeLaserGenAlgorithm::FakeLaserGenAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

FakeLaserGenAlgorithm::~FakeLaserGenAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void FakeLaserGenAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// FakeLaserGenAlgorithm Public API
