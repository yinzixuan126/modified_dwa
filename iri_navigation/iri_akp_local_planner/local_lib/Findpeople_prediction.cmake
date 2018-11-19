FIND_PATH(people_prediction_INCLUDE_DIR iri_geometry.h /usr/include/iridrivers/people_prediction /usr/local/include/iridrivers/people_prediction)

FIND_LIBRARY(people_prediction_LIBRARY
    NAMES people_prediction
    PATHS /usr/lib /usr/local/lib /usr/local/lib/iridrivers) 

IF (people_prediction_INCLUDE_DIR AND people_prediction_LIBRARY)
   SET(people_prediction_FOUND TRUE)
ENDIF (people_prediction_INCLUDE_DIR AND people_prediction_LIBRARY)

IF (people_prediction_FOUND)
   IF (NOT people_prediction_FIND_QUIETLY)
      MESSAGE(STATUS "Found people_prediction library: ${people_prediction_LIBRARY}")
   ENDIF (NOT people_prediction_FIND_QUIETLY)
ELSE (people_prediction_FOUND)
   IF (people_prediction_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find people_prediction library")
   ENDIF (people_prediction_FIND_REQUIRED)
ENDIF (people_prediction_FOUND)

