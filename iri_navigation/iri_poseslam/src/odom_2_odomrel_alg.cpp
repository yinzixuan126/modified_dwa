#include "odom_2_odomrel_alg.h"

Odom2OdomrelAlgorithm::Odom2OdomrelAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

Odom2OdomrelAlgorithm::~Odom2OdomrelAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void Odom2OdomrelAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  this->unlock();
}

// Odom2OdomrelAlgorithm Public API