#include "diff_platform_simulator_alg.h"

DiffPlatformSimulatorAlgorithm::DiffPlatformSimulatorAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

DiffPlatformSimulatorAlgorithm::~DiffPlatformSimulatorAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void DiffPlatformSimulatorAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// DiffPlatformSimulatorAlgorithm Public API
