cmake_minimum_required(VERSION 2.8.3)
project(iri_nav_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED)
# ******************************************************************** 
#                 Add catkin additional components here
# ******************************************************************** 
find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs actionlib_msgs)

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
#  FILES
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
add_action_files(FILES followTarget.action)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs actionlib_msgs # Or other packages containing msgs
)

# ******************************************************************** 
#                 Add the dynamic reconfigure file 
# ******************************************************************** 
#generate_dynamic_reconfigure_options(cfg/<cfg_file_name>.cfg)

# ******************************************************************** 
#                 Add run time dependencies here
# ******************************************************************** 
catkin_package(
#  INCLUDE_DIRS 
#  LIBRARIES 
# ******************************************************************** 
#            Add ROS and IRI ROS run time dependencies
# ******************************************************************** 
 CATKIN_DEPENDS message_runtime geometry_msgs actionlib_msgs
# ******************************************************************** 
#      Add system and labrobotica run time dependencies here
# ******************************************************************** 
#  DEPENDS 
)

###########
## Build ##
###########
# This hack is only to allow the rosmsg tool to locate the messages genererated by the actions, so that the IRI scripts can 
# locate them when adding actions to a node.
add_custom_target(iri_nav_msgs_copy_action_msgs ALL)
add_custom_command(TARGET iri_nav_msgs_copy_action_msgs POST_BUILD 
                   COMMAND rsync -aq ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/msg/ ${CMAKE_CURRENT_SOURCE_DIR}/msg/)
add_dependencies(iri_nav_msgs_copy_action_msgs iri_nav_msgs_generate_messages_cpp)

# ******************************************************************** 
#                   Add the include directories 
# ******************************************************************** 
# include_directories(include)
# include_directories(${catkin_INCLUDE_DIRS})
# include_directories(${<dependency>_INCLUDE_DIR})

## Declare a cpp library
# add_library(${PROJECT_NAME} <list of source files>)

## Declare a cpp executable
# add_executable(${PROJECT_NAME} <list of source files>)

# ******************************************************************** 
#                   Add the libraries
# ******************************************************************** 
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
# target_link_libraries(${PROJECT_NAME} ${<dependency>_LIBRARIES})

# ******************************************************************** 
#               Add message headers dependencies 
# ******************************************************************** 
# add_dependencies(${PROJECT_NAME} <msg_package_name>_generate_messages_cpp)
# ******************************************************************** 
#               Add dynamic reconfigure dependencies 
# ******************************************************************** 
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
