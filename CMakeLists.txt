cmake_minimum_required(VERSION 2.8.3)
project(drive_ros_laserscan_obstacle_generator)

#SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_BUILD_TYPE Debug)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  drive_ros_msgs
  pcl_ros
  laser_geometry
)

# just to make QTCreator include the PCL headers
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp drive_ros_msgs pcl_ros laser_geometry
  # just to make QTCreator include the PCL headers
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # just to make QTCreator include the PCL headers
  ${PCL_INCLUDE_DIRS}
)

################################################################################
# Obstacle generation node
################################################################################

add_executable(laserscan_obstacle_generator_node
    src/laserscan_obstacle_generator.cpp
    src/laserscan_obstacle_generator_node.cpp
    )

add_dependencies(laserscan_obstacle_generator_node ${drive_ros_msgs_generate_messages_cpp} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(laserscan_obstacle_generator_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    )

install(TARGETS laserscan_obstacle_generator_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

################################################################################
# Install other files
################################################################################
# todo: add nodelet support
#install(FILES nodelets.xml
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#    )

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
    )

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
    )

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
    )
