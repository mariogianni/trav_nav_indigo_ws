
#include_directories(${INCLUDE_DIRS} ${EXT_INCLUDE_DIRS})
set(CORELIBS clusterpcl pathplanning pathplanningutils travanalpcl conversionpcl dynamicjoinpcl normalestimation ${catkin_LIBRARIES}  ${PCL_LIBS_DEPS})
#message("-- CORELIBS: ${CORELIBS}")

set(TEST_PROGS 
test  
test_dyn_join_pcl
test_normal
test_dyn_join_pcl_w_normal 
test_clustering
test_traversability 
test_spherical_kdtree 
test_distance_partition 
test_spherical_partition 
test_segmentation 
test_planner 
)

#these are all compiled the same way
set(PROGRAMS ${TEST_PROGS})

foreach(program ${PROGRAMS})
  add_executable(${program} ${program}.cpp)
  target_link_libraries(${program} ${CORELIBS})
  add_dependencies(${program} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
endforeach(program)


