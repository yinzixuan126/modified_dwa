#include "sensors_2_link_alg.h"

Sensors2LinkAlgorithm::Sensors2LinkAlgorithm(void)  
{
  pthread_mutex_init(&this->access_,NULL);
}

Sensors2LinkAlgorithm::~Sensors2LinkAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void Sensors2LinkAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();
  K_mahalanobis_ = new_cfg.K_mahalanobis; // Mahalanobis distance in the outlier detection in loop closure
  this->unlock();
}

// Sensors2LinkAlgorithm Public API