#include "people_simulation_alg.h"

PeopleSimulationAlgorithm::PeopleSimulationAlgorithm(void)
{
}

PeopleSimulationAlgorithm::~PeopleSimulationAlgorithm(void)
{
}

void PeopleSimulationAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// PeopleSimulationAlgorithm Public API