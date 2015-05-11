# MCNP CMAKE FILE
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

INCLUDE_DIRECTORIES(${EMBREE_PATH}/common ${EMBREE_PATH}/build ${EMBREE_PATH}/include ${CMAKE_CURRENT_SOURCE_DIR})

MESSAGE(${CMAKE_CURRENT_SOURCE_DIR})

LIST(APPEND EMDAG_SRC main.cpp embree.cpp)

ADD_EXECUTABLE(embray ${EMDAG_SRC})

ADD_EXECUTABLE(robustness_test emdag_robustness_test.cpp embree.cpp)

ADD_EXECUTABLE(dagmc_preproc dagmc_preproc.cpp DagMC.cpp DagMC.hpp embree.cpp obb_analysis.cpp)

ADD_EXECUTABLE(ray_fire_test ray_fire_test.cc DagMC.cpp DagMC.hpp embree.cpp )

ADD_EXECUTABLE(test_geom test_geom.cc DagMC.cpp DagMC.hpp embree.cpp )

#embree libs
set(EMBREE_LIBRARIES ${EMBREE_PATH}/lib/libembree.so.2)

target_link_libraries(embray ${MOAB_LIBRARIES} ${EMBREE_LIBRARIES} )

include_directories(dagmc_preproc ${MOAB_INCLUDES} /home/shriwise/dagmc_blds/moabs/src/src/)

target_link_libraries(dagmc_preproc ${MOAB_LIBRARIES} ${EMBREE_LIBRARIES})
target_link_libraries(ray_fire_test ${MOAB_LIBRARIES} ${EMBREE_LIBRARIES})
target_link_libraries(test_geom ${MOAB_LIBRARIES} ${EMBREE_LIBRARIES})
target_link_libraries(robustness_test ${MOAB_LIBRARIES} ${EMBREE_LIBRARIES})

ADD_LIBRARY(emdag SHARED DagMC.cpp DagMC.hpp)


INSTALL( TARGETS embray dagmc_preproc ray_fire_test test_geom robustness_test RUNTIME DESTINATION ${INSTALL_BIN_DIR} )
