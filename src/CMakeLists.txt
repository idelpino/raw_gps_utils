
MESSAGE("Starting raw_gps_utils CMakeLists ...")
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

##find dependencies. 
#FIND_PACKAGE(faramotics QUIET) #faramotics is only required for some tests
#IF(faramotics_FOUND)
#	#FIND_PACKAGE(GLUT REQUIRED)
#	FIND_PACKAGE(pose_state_time REQUIRED)
#    MESSAGE("Faramotics Library FOUND: Tests requiring it will be built.")
#ENDIF(faramotics_FOUND)

## include directories
INCLUDE_DIRECTORIES(.)
IF(Ceres_FOUND)
    INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
ENDIF(Ceres_FOUND)
#IF(faramotics_FOUND)
#    INCLUDE_DIRECTORIES(${faramotics_INCLUDE_DIRS})
#ENDIF(faramotics_FOUND)

## headers
SET(HDRS_BASE
    raw_gps_utils.h)
    
SET(HDRS
	satellites_obs.h
	time_stamp.h
)

## sources
SET(SRCS
	satellites_obs.cpp
    time_stamp.cpp
)
    
## create the shared library
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SRCS})

## install library
INSTALL(TARGETS ${PROJECT_NAME}
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib/iri-algorithms
      ARCHIVE DESTINATION lib/iri-algorithms)
        
## install headers         
INSTALL(FILES ${HDRS_BASE} 
      DESTINATION include/iri-algorithms/raw_gps_utils)
INSTALL(FILES ${HDRS} 
      DESTINATION include/iri-algorithms/raw_gps_utils)

## install Find*.cmake
INSTALL(FILES ../Findraw_gps_utils.cmake DESTINATION ${CMAKE_ROOT}/Modules/)

## Build examples & tests
MESSAGE("Building examples and tests.")
ADD_SUBDIRECTORY(examples)
