cmake_minimum_required(VERSION 3.0.2)
project(target_scenario_generator)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(target_attack_goal_node src/target_attack_goal_node.cpp)
add_dependencies(target_attack_goal_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(target_attack_goal_node ${catkin_LIBRARIES})

install(TARGETS target_attack_goal_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
