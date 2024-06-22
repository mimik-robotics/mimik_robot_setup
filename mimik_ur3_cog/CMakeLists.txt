cmake_minimum_required(VERSION 3.1.3)
project(mimik_ur3_cog)

set(CMAKE_CXX_STANDARD 17)

find_package(orocos_kdl)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  eigen_conversions
  kdl_conversions
  moveit_core
  moveit_ros_planning_interface
)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    eigen_conversions
    kdl_conversions
    moveit_core
    moveit_ros_planning_interface
  DEPENDS
    EIGEN3
    orocos_kdl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/cog.cpp
  src/transform_frame.cpp
)

add_executable(pipette_tool_cog src/pipette_tool_cog.cpp)
target_link_libraries(pipette_tool_cog ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(ur3_tool_cog src/ur3_tool_cog.cpp)
target_link_libraries(ur3_tool_cog ${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS
  ${PROJECT_NAME}
  pipette_tool_cog
  ur3_tool_cog
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
