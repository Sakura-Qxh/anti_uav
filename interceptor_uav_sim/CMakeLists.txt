cmake_minimum_required(VERSION 3.0.2)
project(interceptor_uav_sim)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp nav_msgs geometry_msgs visualization_msgs tf2 tf2_geometry_msgs quadrotor_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(interceptor_uav_node src/interceptor_uav_node.cpp)
target_link_libraries(interceptor_uav_node ${catkin_LIBRARIES})
add_dependencies(interceptor_uav_node ${catkin_EXPORTED_TARGETS})

install(TARGETS interceptor_uav_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)