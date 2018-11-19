#include "trajectory_scans_2_pointcloud_alg.h"

TrajectoryScans2PointcloudAlgorithm::TrajectoryScans2PointcloudAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

TrajectoryScans2PointcloudAlgorithm::~TrajectoryScans2PointcloudAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void TrajectoryScans2PointcloudAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  this->unlock();
}

// TrajectoryScans2PointcloudAlgorithm Public API