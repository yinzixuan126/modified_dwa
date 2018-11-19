#include "poseSLAM.h"
#include <stdio.h>

int main(int argc, char *argv[])
{
	uint nSteps=1e6;
	Vector3d d(3);
  d << 0.5, 0, M_PI / 10;
  MatrixXd OdomNoise = 1e-6 * MatrixXd::Identity(3,3);
  MatrixXd LoopNoise = 1e-10 * MatrixXd::Identity(3,3);

	// Initial pose gaussian
  MatrixXd initS = 1e-8 * MatrixXd::Identity(3,3);
  Vector3d initv = MatrixXd::Zero(3,1);

  // Parameters	
  Params Parameters;
  Parameters.matchArea << 3, 3, M_PI / 8;
  Parameters.pdRange.first = 0.1; // If the probability of 2 poses of being closer than 'matchArea' is larger than 'pd.first' we try to create a loop.
  Parameters.pdRange.second = 0.9; // If the probability of 2 poses of being closer than 'matchArea' is larger than 'pd.second' they are close enough for one of them to be redundant
  Parameters.igRange.first = 1; // If the information gain is smaller than 'ig.first' the pose is redundant and it can be overwrite next time slice	
  Parameters.igRange.second = 3; // If the information gain is larger than 'ig.second' we try to create a loop
  Parameters.LoopNoise = LoopNoise;
	Parameters.K_mahalanobis = 100;
	Parameters.ignorePrevious = 1;
	Parameters.min_cov = 1e-12;

  // creating poseSLAM object
  CPoseSLAM pSLAM(initv, initS, Parameters);
  
  // START SIMULATION
  printf("\n---------------------------------------------------------------");
  printf("\n---------------------- START SIMULATION -----------------------");
  printf("\n---------------------------------------------------------------\n\n");
  
  for(uint step = 1; step < nSteps; step++)
  {
    // State Augmentation (always same odometry)
    pSLAM.augmentation(step, d, OdomNoise);
    
    if (step % 100 == 0)
  		printf("step %i - pose: %f, %f, %f\n", step, pSLAM.get_trajectory().back()(0), pSLAM.get_trajectory().back()(1), pSLAM.get_trajectory().back()(2));

    // Create list of link candidates
    //printf("CANDIDATES LIST: \n");
    pSLAM.create_candidates_list();
    //printf("\n");

    // Process all link candidates for loop closure
    while (pSLAM.any_candidate())
    {
    	// Select the most information gain link candidate
      pSLAM.select_best_candidate();
      //std::cout << "---" << std::endl;
      //printf("ig = %f\n", pSLAM.get_candidate_link().iGain);
      
      // Check if the present pose is redundant
      pSLAM.redundant_evaluation();
      
      // Check if we can try loop closure
      if (pSLAM.loop_closure_requeriments())
      {
			  // Sensor call for loop closure
			  MatrixXd LN = LoopNoise;
			  if (pSLAM.try_loop_closure(LN, pSLAM.get_candidate_link().d)) 
        {
          printf("\tlc: [%i %i] ig:%f pd:%f ->  x:%f  y:%f  o:%f\n", 
                  pSLAM.get_candidate_step()+1, 
                  step+1, 
                  pSLAM.get_candidate_link().iGain,
                  pSLAM.get_candidate_link().pd, 
                  pSLAM.get_candidate_link().d(0),  
                  pSLAM.get_candidate_link().d(1),  
                  pSLAM.get_candidate_link().d(2));
          pSLAM.update_candidates_list();
        }
        // else
        // 	printf("Couldn't close the loop!\n");
      }
    }
    // if (!pSLAM.is_redundant())
    // 	printf("State %u\n", pSLAM.get_FF().get_PF().step_2_state(step));
  }
  
  // PRINT RESULTS
  // Print Loops Closed in screen
  std::vector<Loop> LC = pSLAM.get_FF().get_PF().get_loops();
  printf("LOOPS CLOSED: %lu\n", LC.size());
  for (uint i = 0; i < LC.size(); i++)
    printf("\t%i: steps %i - %i || states %i - %i\n", 
    				i + 1,
    				LC[i].p1,
    				LC[i].p2,
    				pSLAM.get_FF().get_PF().step_2_state(LC[i].p1),
    				pSLAM.get_FF().get_PF().step_2_state(LC[i].p2));
}
