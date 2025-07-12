cmake_minimum_required(VERSION 3.5)
project(arm_controller)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  geometry_msgs 
  roscpp 
  sensor_msgs 
  std_msgs
  tf2 
  tf2_ros 
  tf2_geometry_msgs
  DEPENDS EIGEN3
)

file(GLOB_RECURSE SOURCES
  src/*.cpp
)

include_directories(
  include
  include/arm_controller
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
add_executable(${PROJECT_NAME}_node
  app/champ.cpp
  ${SOURCES}
)

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

# Test
add_executable(${PROJECT_NAME}_test
  test/test.cpp
  src/core.cpp
  ${SOURCES}
)
add_dependencies(${PROJECT_NAME}_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_test
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)



install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)