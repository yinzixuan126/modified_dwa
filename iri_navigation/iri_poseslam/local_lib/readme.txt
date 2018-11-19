----------------INSTALLATION-------------------

PoseSLAM requires Eigen 3.2:
- Eigen: http://eigen.tuxfamily.org (Version 3.2 Beta required)

After Eigen installation, create a new folder "build" and run "make" and "make install" into it.

--------------------TEST-----------------------

A test binary will be generated. Run "poseSLAM_test" (/bin folder) and a test of a given odometry will run the pose slam algorithm, result must have 4 Loops Closed:

  LOOPS CLOSED: 4
	1: steps 192 - 0 || states 28 - 0
	2: steps 329 - 83 || states 48 - 12
	3: steps 363 - 115 || states 53 - 16
	4: steps 478 - 181 || states 70 - 26

Also a matlab file will be created with the trajectory result.

--------------ADDITIONAL INFO------------------

There is a ROS package for running the poseSLAM algorithm with real laserScan data. It can be found in: http://devel.iri.upc.edu/pub/labrobotica/ros/iri-ros-pkg/stacks/iri_navigation/trunk/iri_poseslam/
