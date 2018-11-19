#include "iri_fusion_tracks_akp_people_simulation_alg.h"

IriFusionTracksAkpPeopleSimulationAlgorithm::IriFusionTracksAkpPeopleSimulationAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

IriFusionTracksAkpPeopleSimulationAlgorithm::~IriFusionTracksAkpPeopleSimulationAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void IriFusionTracksAkpPeopleSimulationAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// IriFusionTracksAkpPeopleSimulationAlgorithm Public API
