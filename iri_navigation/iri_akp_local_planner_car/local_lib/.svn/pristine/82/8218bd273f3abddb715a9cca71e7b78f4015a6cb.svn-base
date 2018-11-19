# locate the necessary dependencies


INCLUDE_DIRECTORIES(.)
# add the necessary include directories
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

# extra source files
SET(sources_random
	random/rand_gmm.cpp
)
SET(sources_scene_elements
	scene_elements/person_abstract.cpp
    scene_elements/person_bhmip.cpp
    scene_elements/person_behavior.cpp
    scene_elements/robot.cpp
    scene_elements/person_virtual.cpp
)
SET(sources_nav
    #nav/force_reactive_robot_companion.cpp
    #nav/force_reactive_robot_companion_learning.cpp
    nav/plan_local_nav.cpp
)

# extra header files
SET(headers_random
	random/rand_gmm.h
)
SET(headers_scene_elements
	scene_elements/person_abstract.h
    scene_elements/person_bhmip.h
    scene_elements/person_behavior.h
    scene_elements/robot.h
    scene_elements/person_virtual.h
)
SET(headers_nav
    #nav/force_reactive_robot_companion.h
    #nav/force_reactive_robot_companion_learning.h
    nav/plan_local_nav.h
)

# application source files
SET(sources
	iri_geometry.cpp
    scene_abstract.cpp
	prediction_bhmip.cpp
	prediction_behavior.cpp
    scene_sim.cpp
)
# application header files
SET(headers
	iri_geometry.h
	scene_abstract.h
	prediction_bhmip.h
	prediction_behavior.h
    scene_sim.h
)
# create the shared library
ADD_LIBRARY(people_prediction SHARED  ${sources_scene_elements} ${sources}  ${sources_random} ${sources_nav} )
#TARGET_LINK_LIBRARIES( .. )
#target_link_libraries(${PROJECT_NAME} ${position_3d_LIBRARY})


#install 
INSTALL(TARGETS people_prediction
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib/iridrivers
        ARCHIVE DESTINATION lib/iridrivers)

INSTALL(FILES ${headers} DESTINATION include/iridrivers/people_prediction)
INSTALL(FILES ${headers_random} DESTINATION include/iridrivers/people_prediction/random)
INSTALL(FILES ${headers_scene_elements} DESTINATION include/iridrivers/people_prediction/scene_elements)
INSTALL(FILES ${headers_nav} DESTINATION include/iridrivers/people_prediction/nav)


INSTALL(FILES ../Findpeople_prediction.cmake DESTINATION ${CMAKE_ROOT}/Modules/)


ADD_SUBDIRECTORY(examples)
#uncoment to add tests
#ADD_SUBDIRECTORY(test)
