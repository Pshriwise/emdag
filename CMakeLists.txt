####################################################
#
# CMakeLists.txt for top-level EMDAG repository
#
#####################################################

CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
PROJECT ( EMDAG )

ADD_DEFINITIONS(
    -std=c++11 # Or -std=c++0x
)

###########################
#
# FIND DEPENDENCIES
#
##########################
# Needed for FindPyne.cmake
SET ( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_LIST_DIR}/cmake)

FIND_PACKAGE ( MOAB REQUIRED ) 
MESSAGE ( STATUS "Found MOAB Version: " ${MOAB_VERSION} )
  MESSAGE ( STATUS "MOAB_LIBRARIES is " ${MOAB_LIBRARIES})
  MESSAGE ( STATUS "MOAB_INCLUDE_DIRS is " ${MOAB_INCLUDE_DIRS})
INCLUDE_DIRECTORIES( ${MOAB_INCLUDE_DIRS} )

# Embree
INCLUDE_DIRECTORIES( ${EMBREE_PATH}/common )
INCLUDE_DIRECTORIES( ${EMBREE_PATH}/include)

SET ( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})

###################################
#
# Install dirs 
#
###################################

SET( INSTALL_LIB_DIR     lib     CACHE PATH "Installation directory for libraries" )
SET( INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header" )
SET( INSTALL_BIN_DIR     bin     CACHE PATH "Installation directory for executables" )

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#
# Add the src subdirectory
#
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ADD_SUBDIRECTORY(src)


