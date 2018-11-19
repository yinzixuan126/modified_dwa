# modify this line if you WON'T install the library in the system
# -----------------------------------------------------------------
SET(poseslam_local_path ~/iri-lab/labrobotica/algorithms/poseSLAM)
# -----------------------------------------------------------------

FIND_PATH(pslam_INCLUDE_DIR poseSLAM.h
${poseslam_local_path}/src/poseSLAM
/usr/include/iridrivers
/usr/local/include/iridrivers)

FIND_PATH(btree_INCLUDE_DIR btree.h
${poseslam_local_path}/src/BTree
/usr/include/iridrivers
/usr/local/include/iridrivers)

FIND_PATH(flashFilter_INCLUDE_DIR flashFilter.h
${poseslam_local_path}/src/flashFilter
/usr/include/iridrivers
/usr/local/include/iridrivers)

FIND_PATH(gaussian_INCLUDE_DIR gaussian.h
${poseslam_local_path}/src/Gaussian
/usr/include/iridrivers
/usr/local/include/iridrivers)

FIND_PATH(interval_INCLUDE_DIR interval.h
${poseslam_local_path}/src/interval
/usr/include/iridrivers
/usr/local/include/iridrivers)

FIND_PATH(pose_INCLUDE_DIR pose.h
${poseslam_local_path}/src/Pose
/usr/include/iridrivers
/usr/local/include/iridrivers)

FIND_PATH(pose2D_INCLUDE_DIR pose2D.h
${poseslam_local_path}/src/Pose2D
/usr/include/iridrivers
/usr/local/include/iridrivers)

FIND_PATH(poseData_INCLUDE_DIR poseData.h
${poseslam_local_path}/src/PoseData
/usr/include/iridrivers
/usr/local/include/iridrivers)

FIND_PATH(poseFilter_INCLUDE_DIR poseFilter.h
${poseslam_local_path}/src/PoseFilter
/usr/include/iridrivers
/usr/local/include/iridrivers)

SET(poseSLAM_INCLUDE_DIRS ${pslam_INCLUDE_DIR} ${btree_INCLUDE_DIR} ${flashFilter_INCLUDE_DIR} 
  ${gaussian_INCLUDE_DIR} ${interval_INCLUDE_DIR} ${pose_INCLUDE_DIR} ${pose2D_INCLUDE_DIR} 
  ${poseData_INCLUDE_DIR} ${poseFilter_INCLUDE_DIR})

FIND_LIBRARY(poseSLAM_LIBRARY
    NAMES poseSLAM
    PATHS ${poseslam_local_path}/lib /usr/lib /usr/local/lib /usr/local/lib/iridrivers) 

IF (poseSLAM_INCLUDE_DIRS AND poseSLAM_LIBRARY)
   SET(poseSLAM_FOUND TRUE)
ENDIF (poseSLAM_INCLUDE_DIRS AND poseSLAM_LIBRARY)

IF (poseSLAM_FOUND)
   IF (NOT poseSLAM_FIND_QUIETLY)
      MESSAGE(STATUS "Found poseSLAM: ${poseSLAM_LIBRARY}")
   ENDIF (NOT poseSLAM_FIND_QUIETLY)
ELSE (poseSLAM_FOUND)
   IF (poseSLAM_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find poseSLAM")
   ENDIF (poseSLAM_FIND_REQUIRED)
ENDIF (poseSLAM_FOUND)

