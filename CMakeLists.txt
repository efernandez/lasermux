cmake_minimum_required(VERSION 2.8.3)
project(lasermux)

find_package(catkin REQUIRED
    roscpp
    laser_geometry
    tf
    sensor_msgs
    pcl_ros
    pcl_conversions)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(lasermux src/virtual_laser.cpp)
target_link_libraries(lasermux ${catkin_LIBRARIES})

add_executable(lasermux_node src/lasermux_node.cpp)
target_link_libraries(lasermux_node lasermux ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS lasermux lasermux_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lasermux.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
