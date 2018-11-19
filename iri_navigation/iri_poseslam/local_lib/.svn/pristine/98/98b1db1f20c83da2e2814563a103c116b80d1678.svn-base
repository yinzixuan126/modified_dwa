# Pre-requisites about cmake itself
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0005 NEW) 
  cmake_policy(SET CMP0003 NEW) 
endif(COMMAND cmake_policy)

# The project name and the type of project
PROJECT(poseSLAM)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

IF ("${INSTALL_DIR}" STREQUAL "")
  MESSAGE ("[INSTALL] set install_dir to /usr/local")
  SET(CMAKE_INSTALL_PREFIX /usr/local)
ELSE ()
  MESSAGE ("[INSTALL] set install_dir to input ${INSTALL_DIR}")
  SET(CMAKE_INSTALL_PREFIX ${INSTALL_DIR})
ENDIF ()

IF (NOT CMAKE_BUILD_TYPE)
 SET(CMAKE_BUILD_TYPE "RELEASE") 
ENDIF (NOT CMAKE_BUILD_TYPE)

SET(CMAKE_CXX_FLAGS_DEBUG "-g -Wall")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3")

ADD_SUBDIRECTORY(src)

FIND_PACKAGE(Doxygen)

FIND_PATH(IRI_DOC_DIR doxygen.conf ${CMAKE_SOURCE_DIR}/doc/iri_doc/)
IF (IRI_DOC_DIR)
  ADD_CUSTOM_TARGET (doc ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/doc/iri_doc/doxygen.conf)
ELSE (IRI_DOC_DIR)
  ADD_CUSTOM_TARGET (doc ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/doc/doxygen.conf)
ENDIF (IRI_DOC_DIR)

ADD_CUSTOM_TARGET (distclean @echo cleaning cmake files)

IF (UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "distribution clean"
    COMMAND make ARGS clean
    COMMAND rm ARGS -rf ${CMAKE_SOURCE_DIR}/build/*

    TARGET  distclean
  )
ELSE(UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "distclean only implemented in unix"
    TARGET  distclean
  )
ENDIF(UNIX)

ADD_CUSTOM_TARGET (uninstall @echo uninstall package)

IF (UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "uninstall package"
    COMMAND xargs ARGS rm < install_manifest.txt

    TARGET  uninstall
  )
ELSE(UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "uninstall only implemented in unix"
    TARGET  uninstall
  )
ENDIF(UNIX)

IF (UNIX)
  SET(CPACK_PACKAGE_FILE_NAME "iri-${PROJECT_NAME}-dev-${CPACK_PACKAGE_VERSION}${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
  SET(CPACK_PACKAGE_NAME "iri-${PROJECT_NAME}-dev")
  SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Part of IRI-laboratory libraries. More information at http://wikiri.upc.es/index.php/Robotics_Lab")
  SET(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
  SET(CPACK_GENERATOR "DEB")
  SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "galenya - labrobotica@iri.upc.edu")
  SET(CPACK_SET_DESTDIR "ON")  # Necessary because of the absolute install paths

  INCLUDE(CPack)
ELSE(UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "packaging only implemented in unix"
    TARGET  uninstall
  )
ENDIF(UNIX)

