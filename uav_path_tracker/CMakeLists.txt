cmake_minimum_required(VERSION 3.0.2)
project(uav_path_tracker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(path_tracker_node src/path_tracker_node.cpp)
target_link_libraries(path_tracker_node ${catkin_LIBRARIES} /usr/local/lib/libboost_system.so)

install(TARGETS path_tracker_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
