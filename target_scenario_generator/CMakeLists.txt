cmake_minimum_required(VERSION 3.0.2)
project(target_scenario_generator)

add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(target_motion_node
  src/target_motion_node.cpp
)

add_dependencies(target_motion_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(target_motion_node
  ${catkin_LIBRARIES}
  /usr/local/lib/libboost_system.so
)

install(TARGETS target_motion_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
