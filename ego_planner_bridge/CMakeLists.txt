cmake_minimum_required(VERSION 3.0.2)
project(ego_planner_bridge)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(goal_to_waypoints_bridge_node src/goal_to_waypoints_bridge_node.cpp)
add_dependencies(goal_to_waypoints_bridge_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(goal_to_waypoints_bridge_node ${catkin_LIBRARIES})

add_executable(odom_truth_bridge_node src/odom_truth_bridge_node.cpp)
add_dependencies(odom_truth_bridge_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_truth_bridge_node ${catkin_LIBRARIES})

install(TARGETS goal_to_waypoints_bridge_node odom_truth_bridge_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)