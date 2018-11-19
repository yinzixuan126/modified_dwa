----------------INSTALLATION-------------------

iri_poseslam package requires the PoseSLAM algorithm installed:  http://devel.iri.upc.edu/pub/labrobotica/algorithms/poseSLAM/
(don't forget its libraries dependencies)

It also requires csm package. Install it doing:

    $ sudo apt-get install libgsl0-dev
    $ cd your-ros-workspace
    $ git clone https://github.com/ccny-ros-pkg/scan_tools.git


--------------ADDITIONAL INFO------------------

The 'poseSLAM' node works with odometry messages (not sensor data), it needs a "translate" node for each type of sensor data. Only scan laser "translator" is implemented: 'scans_2_odom' node.

The 'results_publisher' node publish the acumulated pointcloud of all laser scans of a trajectory for 'rviz' visualization.

Also exist the 'trajectory_2_markers' node that publish all poses and covariances of the trajectory for 'rviz' visualization.

A rviz config file 'poseSLAM.launch' is given as example and may help understanding function of each node.
