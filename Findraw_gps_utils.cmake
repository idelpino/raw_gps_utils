#edit the following line to add the librarie's header files
FIND_PATH(
    raw_gps_utils_INCLUDE_DIRS
    NAMES raw_gps_utils.h
    PATHS /usr/local/include/iri-algorithms/raw_gps_utils)
#change INCLUDE_DIRS to its parent directory
get_filename_component(raw_gps_utils_INCLUDE_DIRS ${raw_gps_utils_INCLUDE_DIRS} DIRECTORY)
MESSAGE(STATUS "raw_gps_utils_INCLUDE_DIRS: ${raw_gps_utils_INCLUDE_DIRS}")

FIND_LIBRARY(
    raw_gps_utils_LIBRARY
    NAMES raw_gps_utils
    PATHS /usr/lib /usr/local/lib /usr/local/lib/iri-algorithms) 
    
MESSAGE(STATUS "raw_gps_utils_LIBRARY: ${raw_gps_utils_LIBRARY}")

IF (raw_gps_utils_INCLUDE_DIRS AND raw_gps_utils_LIBRARY)
   SET(raw_gps_utils_FOUND TRUE)
ENDIF (raw_gps_utils_INCLUDE_DIRS AND raw_gps_utils_LIBRARY)

IF (raw_gps_utils_FOUND)
   IF (NOT raw_gps_utils_FIND_QUIETLY)
      MESSAGE(STATUS "Found raw_gps_utils: ${raw_gps_utils_LIBRARY}")
   ENDIF (NOT raw_gps_utils_FIND_QUIETLY)
ELSE (raw_gps_utils_FOUND)
   IF (raw_gps_utils_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find raw_gps_utils")
   ENDIF (raw_gps_utils_FIND_REQUIRED)
ENDIF (raw_gps_utils_FOUND)

