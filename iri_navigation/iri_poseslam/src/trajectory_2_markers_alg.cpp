#include "trajectory_2_markers_alg.h"

Trajectory2MarkersAlgorithm::Trajectory2MarkersAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

Trajectory2MarkersAlgorithm::~Trajectory2MarkersAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void Trajectory2MarkersAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  
  this->unlock(); 
}