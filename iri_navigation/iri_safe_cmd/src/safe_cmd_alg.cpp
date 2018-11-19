#include "safe_cmd_alg.h"

SafeCmdAlgorithm::SafeCmdAlgorithm(void)
{
}

SafeCmdAlgorithm::~SafeCmdAlgorithm(void)
{
}

void SafeCmdAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// SafeCmdAlgorithm Public API