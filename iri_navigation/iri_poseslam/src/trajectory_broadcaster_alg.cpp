#include "trajectory_broadcaster_alg.h"

TrajectoryBroadcasterAlgorithm::TrajectoryBroadcasterAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

TrajectoryBroadcasterAlgorithm::~TrajectoryBroadcasterAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void TrajectoryBroadcasterAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();
  this->unlock();
}

// TrajectoryBroadcasterAlgorithm Public API