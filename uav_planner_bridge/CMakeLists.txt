cmake_minimum_required(VERSION 3.0.2)
project(uav_planner_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  sensor_msgs
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(planner_bridge_node src/planner_bridge_node.cpp)
target_link_libraries(planner_bridge_node ${catkin_LIBRARIES} /usr/local/lib/libboost_system.so)

add_executable(polynomial_trajectory_to_path_node src/polynomial_trajectory_to_path_node.cpp)
add_dependencies(polynomial_trajectory_to_path_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(polynomial_trajectory_to_path_node ${catkin_LIBRARIES} /usr/local/lib/libboost_system.so)

add_executable(planning_goal_to_odometry_node src/planning_goal_to_odometry_node.cpp)
add_dependencies(planning_goal_to_odometry_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(planning_goal_to_odometry_node ${catkin_LIBRARIES} /usr/local/lib/libboost_system.so)

install(TARGETS planner_bridge_node polynomial_trajectory_to_path_node planning_goal_to_odometry_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
