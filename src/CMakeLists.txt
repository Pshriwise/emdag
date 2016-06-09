# MCNP CMAKE FILE
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

INCLUDE_DIRECTORIES(${EMBREE_INCLUDE_DIRS}  ${CMAKE_CURRENT_SOURCE_DIR})

MESSAGE(${CMAKE_CURRENT_SOURCE_DIR})

LIST(APPEND EMDAG_SRC main.cpp embree.cpp)

ADD_EXECUTABLE(robustness_test emdag_robustness_test.cpp embree.cpp)

ADD_EXECUTABLE(dagmc_preproc dagmc_preproc.cpp DagMC.cpp DagMC.hpp embree.cpp obb_analysis.cpp)

ADD_EXECUTABLE(ray_fire_test ray_fire_test.cc DagMC.cpp DagMC.hpp embree.cpp )

ADD_EXECUTABLE(test_geom test_geom.cc DagMC.cpp DagMC.hpp embree.cpp )

ADD_EXECUTABLE(pt_vol_test pt_vol_test.cc DagMC.cpp DagMC.hpp embree.cpp)

INCLUDE_DIRECTORIES(dagmc_preproc ${MOAB_INCLUDES} ${EMBREE_INCLUDE_DIRS})

TARGET_LINK_LIBRARIES(dagmc_preproc ${MOAB_LIBRARIES} ${EMBREE_LIBRARY})
TARGET_LINK_LIBRARIES(ray_fire_test ${MOAB_LIBRARIES} ${EMBREE_LIBRARY})
TARGET_LINK_LIBRARIES(test_geom ${MOAB_LIBRARIES} ${EMBREE_LIBRARY})
TARGET_LINK_LIBRARIES(robustness_test ${MOAB_LIBRARIES} ${EMBREE_LIBRARY})
TARGET_LINK_LIBRARIES(pt_vol_test ${MOAB_LIBRARIES} ${EMBREE_LIBRARY})

ADD_LIBRARY(emdag SHARED DagMC.cpp DagMC.hpp embree.cpp embree.hpp)


INSTALL( TARGETS robustness_test  dagmc_preproc ray_fire_test pt_vol_test test_geom RUNTIME DESTINATION bin )

INSTALL ( FILES DagMC.hpp DESTINATION include )
INSTALL ( FILES embree.hpp DESTINATION include )

INSTALL ( TARGETS emdag LIBRARY DESTINATION lib )


set(CONF_INCLUDE_DIRS ${MOAB_INCLUDE_DIRS} ${EMBREE_INCLUDE_DIRS})

CONFIGURE_FILE (MOABConfig.cmake.in "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/MOABConfig.cmake" @ONLY)

INSTALL ( FILES   "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/MOABConfig.cmake" DESTINATION lib )
