# edit the following line to add all the source code files of the library
SET(sources ./poseSLAM/poseSLAM.cpp ./BTree/btree.cpp ./flashFilter/flashFilter.cpp ./Gaussian/gaussian.cpp ./interval/interval.cpp ./Pose/pose.cpp ./Pose2D/pose2D.cpp ./PoseData/poseData.cpp ./PoseFilter/poseFilter.cpp)

# edit the following line to add all the header files of the library
SET(headers ./poseSLAM/poseSLAM.h ./BTree/btree.h ./flashFilter/flashFilter.h ./Gaussian/gaussian.h ./interval/interval.h ./Pose/pose.h ./Pose2D/pose2D.h ./PoseData/poseData.h ./PoseFilter/poseFilter.h)

# edit the following line to add the necessary include directories
INCLUDE_DIRECTORIES(./poseSLAM ./BTree ./flashFilter ./Gaussian ./interval ./Pose ./Pose2D ./PoseData ./PoseFilter)

# create the shared library
ADD_LIBRARY(poseSLAM SHARED ${sources})

# link necessary libraries
INSTALL(TARGETS poseSLAM
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib/iridrivers
        ARCHIVE DESTINATION lib/iridrivers)

INSTALL(FILES ${headers} DESTINATION include/iridrivers)

INSTALL(FILES ../FindposeSLAM.cmake DESTINATION ${CMAKE_ROOT}/Modules/)

ADD_SUBDIRECTORY(examples)
