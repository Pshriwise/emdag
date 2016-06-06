# Try to find Embree

find_path( EMBREE_CMAKE_CONFIG NAMES embree-config.cmake
  PATHS ENV LD_LIBRARY_PATH
  PATH_SUFFIXES lib Lib cmake/embree-2.6.0/
  NO_DEFAULT_PATH
  )

MESSAGE ( STATUS "Found MOAB in ${MOAB_CMAKE_CONFIG}" )

INCLUDE ( ${EMBREE_CMAKE_CONFIG}/embree-config.cmake )
