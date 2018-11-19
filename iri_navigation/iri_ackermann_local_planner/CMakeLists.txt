cmake_minimum_required(VERSION 2.8.3)
project(iri_ackermann_local_planner)

## Find catkin macros and libraries
find_package(catkin REQUIRED)
# ******************************************************************** 
#                 Add catkin additional components here
# ******************************************************************** 
find_package(catkin REQUIRED COMPONENTS std_msgs nav_msgs visualization_msgs geometry_msgs nav_core pluginlib tf costmap_2d voxel_grid angles pcl_ros base_local_planner cmake_modules)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)

# ******************************************************************** 
#           Add system and labrobotica dependencies here
# ******************************************************************** 
# find_package(<dependency> REQUIRED)
find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

find_package(iriutils REQUIRED)

# ******************************************************************** 
#           Add topic, service and action definition here
# ******************************************************************** 
## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs  # Or other packages containing msgs
#)

# ******************************************************************** 
#                 Add the dynamic reconfigure file 
# ******************************************************************** 
generate_dynamic_reconfigure_options(cfg/AckermannLocalPlanner.cfg)

# ******************************************************************** 
#                 Add run time dependencies here
# ******************************************************************** 
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES iri_ackermann_local_planner
# ******************************************************************** 
#            Add ROS and IRI ROS run time dependencies
# ******************************************************************** 
 CATKIN_DEPENDS std_msgs nav_msgs visualization_msgs geometry_msgs nav_core pluginlib tf costmap_2d voxel_grid angles pcl_ros base_local_planner
# ******************************************************************** 
#      Add system and labrobotica run time dependencies here
# ******************************************************************** 
#  DEPENDS 
)

###########
## Build ##
###########

# ******************************************************************** 
#                   Add the include directories 
# ******************************************************************** 
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${iriutils_INCLUDE_DIR})

## Declare a cpp library
add_library(${PROJECT_NAME} src/ackermann_odom_helper.cpp src/ackermann_planner_util.cpp src/ackermann_planner_ros src/ackermann_planner.cpp src/ackermann_trajectory_generator.cpp src/heading_cost_function.cpp src/ackermann_trajectory_search.cpp)

## Declare a cpp executable
# add_executable(${PROJECT_NAME} <list of source files>)

# ******************************************************************** 
#                   Add the libraries
# ******************************************************************** 
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${iriutils_LIBRARIES})

# ******************************************************************** 
#               Add message headers dependencies 
# ******************************************************************** 
# add_dependencies(${PROJECT_NAME} <msg_package_name>_cpp)
# ******************************************************************** 
#               Add dynamic reconfigure dependencies 
# ******************************************************************** 
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(foo_node
#   ${catkin_LIBRARIES}
# )

