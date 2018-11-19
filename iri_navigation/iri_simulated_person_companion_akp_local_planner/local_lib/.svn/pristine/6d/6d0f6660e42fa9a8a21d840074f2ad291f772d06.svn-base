SET(Boost_USE_STATIC_LIBS ON)
#set(Boost_USE_MULTITHREADED OFF)
#set(Boost_USE_STATIC_RUNTIME OFF) 
FIND_PACKAGE(Boost COMPONENTS unit_test_framework REQUIRED )
LINK_DIRECTORIES ( ${Boost_LIBRARY_DIRS} )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIRS})
MESSAGE(STATUS "Boost_LIBRARIES=${Boost_LIBRARIES}")

ADD_EXECUTABLE(test_geometry  test_geometry.cpp)
TARGET_LINK_LIBRARIES(test_geometry people_prediction ${Boost_LIBRARIES})

ADD_EXECUTABLE(test_person  test_person.cpp)
TARGET_LINK_LIBRARIES(test_person people_prediction ${Boost_LIBRARIES})

ADD_EXECUTABLE(test_scene  test_scene.cpp)
TARGET_LINK_LIBRARIES(test_scene people_prediction ${Boost_LIBRARIES})

