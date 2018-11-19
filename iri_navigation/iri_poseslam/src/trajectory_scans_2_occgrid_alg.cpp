#include "trajectory_scans_2_occgrid_alg.h"

TrajectoryScans2OccGridAlgorithm::TrajectoryScans2OccGridAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

TrajectoryScans2OccGridAlgorithm::~TrajectoryScans2OccGridAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void TrajectoryScans2OccGridAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  this->unlock();
}

// TrajectoryScans2OccGridAlgorithm Public API