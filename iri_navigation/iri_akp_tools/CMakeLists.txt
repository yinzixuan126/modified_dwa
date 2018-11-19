cmake_minimum_required(VERSION 2.8.3)
project(iri_akp_tools)

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
  )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# ******************************************************************** 
#           Add system and labrobotica dependencies here
# ******************************************************************** 
# find_package(<dependency> REQUIRED)

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

# ******************************************************************** 
#                 Add run time dependencies here
# ******************************************************************** 
catkin_package(
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
# include_directories(${<dependency>_INCLUDE_DIR})

## Declare a cpp library
# add_library(${PROJECT_NAME} <list of source files>)

## Declare a cpp executable
add_executable(diff_platform_simulator src/diff_platform_simulator_alg.cpp src/diff_platform_simulator_alg_node.cpp)
add_executable(fake_laser_gen src/fake_laser_gen_alg.cpp src/fake_laser_gen_alg_node.cpp)

# ******************************************************************** 
#                   Add the libraries
# ******************************************************************** 
target_link_libraries(diff_platform_simulator ${catkin_LIBRARIES})
target_link_libraries(fake_laser_gen ${catkin_LIBRARIES})
# target_link_libraries(${PROJECT_NAME} ${<dependency>_LIBRARY})

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
