cmake_minimum_required(VERSION 2.8.3)
project(iri_simulated_person_companion_akp_local_planner)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

## Find catkin macros and libraries
find_package(catkin REQUIRED)
# ******************************************************************** 
#                 Add catkin additional components here
# ******************************************************************** 
find_package(catkin REQUIRED COMPONENTS iri_base_algorithm sensor_msgs nav_msgs iri_perception_msgs pluginlib nav_core costmap_2d visualization_msgs tf std_msgs)

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

# IRI library, may be installed or not:
find_package(people_prediction QUIET)
IF(people_prediction_FOUND)
    MESSAGE("people_prediction Library FOUND:" ${people_prediction_LIBRARY})
ELSE(people_prediction_FOUND)  
    ADD_SUBDIRECTORY(local_lib)
    MESSAGE("people_prediction Library NOT FOUND: Including local library:")
    SET(people_prediction_INCLUDE_DIR ./local_lib/src/)    
    SET(people_prediction_LIBRARY ${CATKIN_DEVEL_PREFIX}/lib/libpeople_prediction.so)
    MESSAGE("people_prediction Library:" ${people_prediction_LIBRARY})
    MESSAGE("people_prediction include directories:" ${people_prediction_INCLUDE_DIR})
ENDIF(people_prediction_FOUND)

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
generate_dynamic_reconfigure_options(cfg/AkpLocalPlanner.cfg)

# ******************************************************************** 
#                 Add run time dependencies here
# ******************************************************************** 
catkin_package(
#  INCLUDE_DIRS 
  LIBRARIES iri_simulated_person_companion_akp_local_planner
# ******************************************************************** 
#            Add ROS and IRI ROS run time dependencies
# ******************************************************************** 
 CATKIN_DEPENDS iri_base_algorithm sensor_msgs nav_msgs iri_perception_msgs pluginlib nav_core costmap_2d visualization_msgs tf std_msgs
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
include_directories(${people_prediction_INCLUDE_DIR} ${EIGEN_INCLUDE_DIR})


## Declare a cpp library
add_library(${PROJECT_NAME} src/akp_local_planner_alg.cpp src/akp_local_planner_alg_node.cpp)

## Declare a cpp executable
#add_executable(${PROJECT_NAME} src/akp_local_planner_alg.cpp src/akp_local_planner_alg_node.cpp)

# ******************************************************************** 
#                   Add the libraries
# ******************************************************************** 
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${people_prediction_LIBRARY})
# target_link_libraries(${PROJECT_NAME} ${<dependency>_LIBRARY})

# ******************************************************************** 
#               Add message headers dependencies 
# ******************************************************************** 
# add_dependencies(${PROJECT_NAME} <msg_package_name>_generate_messages_cpp)
# ******************************************************************** 
#               Add dynamic reconfigure dependencies 
# ******************************************************************** 
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
