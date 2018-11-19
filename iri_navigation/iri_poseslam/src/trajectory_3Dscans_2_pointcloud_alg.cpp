#include "trajectory_3Dscans_2_pointcloud_alg.h"

Trajectory3DScans2PointcloudAlgorithm::Trajectory3DScans2PointcloudAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

Trajectory3DScans2PointcloudAlgorithm::~Trajectory3DScans2PointcloudAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void Trajectory3DScans2PointcloudAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  this->unlock();
}

// Trajectory3DScans2PointcloudAlgorithm Public API