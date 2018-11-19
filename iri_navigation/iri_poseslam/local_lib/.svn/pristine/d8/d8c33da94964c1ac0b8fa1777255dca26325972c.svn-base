#include "poseData.h"

CPoseData::CPoseData()
{
}

CPoseData::CPoseData(const CPoseData& PD)
  :
  mean(PD.mean),
  S(PD.S),
  CS(PD.CS),
  id(PD.id),
  isInterval(PD.isInterval),
  intervalMean(PD.intervalMean),
  intervalS(PD.intervalS),
  intervalCS(PD.intervalCS),
  intervalId(PD.intervalId)
{
}

CPoseData::CPoseData(CPose& newMean, const MatrixXd& newS, const MatrixXd& newCS, const uint newId)
  :
  mean(newMean),
  S(newS),
  CS(newCS),
  id(newId),
  isInterval(false),
  intervalMean(newMean.get_vector(), newMean.get_vector()),
  intervalS(newS, newS),
  intervalCS(newCS, newCS)
{
  intervalId.push_back(newId);
}

CPoseData::CPoseData(const VectorXd& newMean, const MatrixXd& newS, const MatrixXd& newCS, const uint newId)
  :
  mean(newMean),
  S(newS),
  CS(newCS),
  id(newId),
  isInterval(false),
  intervalMean(newMean, newMean),
  intervalS(newS, newS),
  intervalCS(newCS, newCS)
{
  CPose nMean(newMean);
  mean=nMean;
  intervalId.push_back(newId);
}


CPoseData::CPoseData(const CInterval& iMean, const CInterval& iS, const CInterval& iCS, const std::vector<uint>& iId)
  :
  isInterval(true),
  intervalMean(iMean),
  intervalS(iS),
  intervalCS(iCS),
  intervalId(iId)
{
}

CPoseData::~CPoseData()
{
}

//GETS
CPose CPoseData::get_mean() const
{
  return mean;
}

MatrixXd CPoseData::get_S() const
{
  return S;
}

MatrixXd CPoseData::get_CS() const
{
  return CS;
}

uint CPoseData::get_id() const
{
  return id;
}

CGaussian CPoseData::get_gaussian() const
{
  CGaussian gauss(mean.get_vector(),S);
  return gauss;
}

CInterval CPoseData::get_interval_mean() const
{
  return intervalMean;
}

CInterval CPoseData::get_interval_S() const
{
  return intervalS;
}

CInterval CPoseData::get_interval_CS() const
{
  return intervalCS;
}

std::vector<uint> CPoseData::get_interval_id() const
{
  return intervalId;
}

bool CPoseData::is_interval() const
{
  return isInterval;
}


//SETS
void CPoseData::set_mean(const CPose& newMean)
{
  mean = newMean;
  set_not_interval();
}

void CPoseData::set_S(const MatrixXd& newS)
{
  S = newS;
  set_not_interval();
}

void CPoseData::set_CS(const MatrixXd& newCS)
{
  CS = newCS;
  set_not_interval();
}

void CPoseData::set_id(const uint& newId)
{
  id = newId;
  set_not_interval();
}

void CPoseData::set_all(const CPose& newMean,const MatrixXd& newS,const MatrixXd& newCS)
{
  mean = newMean;
  S = newS;
  CS = newCS;
  set_not_interval();
}

void CPoseData::set_update(const CPose& newMean,const MatrixXd& updateS,const MatrixXd& newCS)
{
  mean = newMean;
  S += updateS;
  CS = newCS;
  set_not_interval();
}

void CPoseData::set_all(const VectorXd& newMean,const MatrixXd& newS,const MatrixXd& newCS)
{
  CPose nMean(newMean);
  mean = nMean;
  S = newS;
  CS = newCS;
  set_not_interval();
}

void CPoseData::set_update(const VectorXd& newMean,const MatrixXd& updateS,const MatrixXd& newCS)
{
  CPose nMean(newMean);
  mean = nMean;
  S += updateS;
  CS = newCS;
  set_not_interval();
}

void CPoseData::set_not_interval()
{
  isInterval=false;
  intervalMean.set_interval(mean.get_vector(), mean.get_vector());
  intervalS.set_interval(S, S);
  intervalCS.set_interval(CS, CS);
  intervalId.clear();
  intervalId.push_back(id);
}

// METHODS

CPoseData CPoseData::pd_union(const CPoseData& PD1,const CPoseData& PD2) const
{
  
  CInterval iMean(PD1.intervalMean.interval_union(PD1.intervalMean, PD2.intervalMean));
  CInterval iS(PD1.intervalS.interval_union(PD1.intervalS, PD2.intervalS));
  CInterval iCS(PD1.intervalCS.interval_union(PD1.intervalCS, PD2.intervalCS));
  
  std::vector<uint> iId;
  iId.reserve(PD1.intervalId.size() + PD2.intervalId.size());
  iId.insert(iId.begin(), PD1.intervalId.begin(), PD1.intervalId.end());
  iId.insert(iId.end(), PD2.intervalId.begin(), PD2.intervalId.end());
  
  CPoseData PDunion(iMean, iS, iCS, iId);
  
  return PDunion;
}

void CPoseData::print_pd() const
{

  if (isInterval)
  {
    printf("\npd IS INTERVAL:\n");
    printf("\nintervalMean:\n");
    intervalMean.print_interval();
    printf("\nintervalS:\n");
    intervalS.print_interval();
    printf("\nintervalCS:\n");
    intervalCS.print_interval();
    printf("\nintervalId - %lu steps:\n",intervalId.size());
    for (uint i = 0; i < intervalId.size(); i++)
      printf(" %u,",intervalId[i]);
    printf("\n\n");  
  }
  else
  {
    printf("\npd IS NOT INTERVAL:\n");
    printf("\nmean:\n");
    std::cout << mean.get_vector() << std::endl;
    printf("\nS:\n");
    std::cout << S << std::endl;
    printf("\nCS:\n");
    std::cout << CS << std::endl;
    printf("\nId:\n%u",id);
	  printf("\n\n");
  }
}
