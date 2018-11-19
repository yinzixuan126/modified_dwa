cmake_minimum_required(VERSION 2.8.3)
project(iri_akp_tools_companion)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

## Find catkin macros and libraries
find_package(catkin REQUIRED)
# ******************************************************************** 
#                 Add catkin additional components here
# ******************************************************************** 
find_package(catkin REQUIRED COMPONENTS 
  iri_base_algorithm 
  geometry_msgs 
  nav_msgs 
  sensor_msgs
  tf
  iri_perception_msgs
  pluginlib 
  nav_core 
  costmap_2d 
  visualization_msgs 
  std_msgs
  )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# ******************************************************************** 
#           Add system and labrobotica dependencies here
# ******************************************************************** 
# find_package(<dependency> REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen QUIET)
if(Eigen_FOUND)
  MESSAGE("Eigen found in CMake modules:" ${Eigen_INCLUDE_DIRS})
ELSE(Eigen_FOUND)  
    #INCLUDE_DIRECTORIES(/usr/include/eigen3)
    #INCLUDE_DIRECTORIES(/usr/local/include/eigen3)
    MESSAGE("Eigen not found, adding manually directories. BE CAREFULL to have installed eigen there")
    SET(Eigen_INCLUDE_DIRS /usr/include/eigen3 /usr/local/include/eigen3)    
ENDIF(Eigen_FOUND)
INCLUDE_DIRECTORIES(${Eigen_INCLUDE_DIRS})


# ******************************************************************** 
#           Add topic, service and action definition here
# ******************************************************************** 
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

# ******************************************************************** 
#                 Add the dynamic reconfigure file 
# ******************************************************************** 
#generate_dynamic_reconfigure_options(cfg/IriAkpTools.cfg)
generate_dynamic_reconfigure_options(cfg/DiffPlatformSimulator.cfg cfg/FakeLaserGen.cfg)
#cfg/FakeAkpLocalPlanner.cfg
# ******************************************************************** 
#                 Add run time dependencies here
# ******************************************************************** 
catkin_package(
 LIBRARIES iri_akp_local_planner
#  INCLUDE_DIRS 
#  LIBRARIES 
# ******************************************************************** 
#            Add ROS and IRI ROS run time dependencies
# ******************************************************************** 
 CATKIN_DEPENDS 
 iri_base_algorithm 
 geometry_msgs 
 nav_msgs 
 sensor_msgs 
 tf
# iri_perception_msgs 
# pluginlib 
# nav_core 
# costmap_2d 
# visualization_msgs
# std_msgs
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
#include_directories(${people_prediction_INCLUDE_DIR}  ${EIGEN_INCLUDE_DIR})
# include_directories(${<dependency>_INCLUDE_DIR})

## Declare a cpp library
# add_library(${PROJECT_NAME} <list of source files>)

## Declare a cpp executable
add_executable(diff_platform_simulator src/diff_platform_simulator_alg.cpp src/diff_platform_simulator_alg_node.cpp)
add_executable(fake_laser_gen src/fake_laser_gen_alg.cpp src/fake_laser_gen_alg_node.cpp)
#add_executable(fake_akp_local_planner_alg src/fake_akp_local_planner_alg.cpp src/fake_akp_local_planner_alg_node.cpp)

# ******************************************************************** 
#                   Add the libraries
# ******************************************************************** 
target_link_libraries(diff_platform_simulator ${catkin_LIBRARIES})
target_link_libraries(fake_laser_gen ${catkin_LIBRARIES})
#target_link_libraries(fake_akp_local_planner ${catkin_LIBRARIES})
# target_link_libraries(${PROJECT_NAME} ${<dependency>_LIBRARY})
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${people_prediction_LIBRARY})

# ******************************************************************** 
#               Add message headers dependencies 
# ******************************************************************** 
# add_dependencies(${PROJECT_NAME} <msg_package_name>_generate_messages_cpp)
message(xxxsadsa ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(diff_platform_simulator 
  geometry_msgs_generate_messages_cpp 
  nav_msgs_generate_messages_cpp 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )
add_dependencies(fake_laser_gen 
  sensor_msgs_generate_messages_cpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )
# ******************************************************************** 
#               Add dynamic reconfigure dependencies 
# ********************************************************************
