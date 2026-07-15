cmake_minimum_required(VERSION 3.0.2)
project(experiment_recorder)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(experiment_recorder_node src/experiment_recorder_node.cpp)
add_dependencies(experiment_recorder_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(experiment_recorder_node ${catkin_LIBRARIES})

install(TARGETS experiment_recorder_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

