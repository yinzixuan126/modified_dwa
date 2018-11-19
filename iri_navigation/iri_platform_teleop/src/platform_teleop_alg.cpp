#include "platform_teleop_alg.h"

PlatformTeleopAlgorithm::PlatformTeleopAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

PlatformTeleopAlgorithm::~PlatformTeleopAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void PlatformTeleopAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// PlatformTeleopAlgorithm Public API
