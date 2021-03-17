cmake_minimum_required(VERSION 2.8.3)
project(drone_explorer)

set(CMAKE_BUILD_TYPE "DEBUG")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g --debug-cpp --trace-expand")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128 -O1 -Wall -g --debug")
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  plan_env
  mpc_opt
  traj_utils
  message_generation
  mavros
  pcl_conversions
  pcl_ros
  sensor_msgs
  octomap_msgs
  hagen_msgs
  octomap_ros
  bspline_opt
  rebound_opt
  laser_geometry
  path_searching
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Bspline.msg
  )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  hagen_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES state_machine
 CATKIN_DEPENDS plan_env  mpc_opt traj_utils message_runtime bspline_opt rebound_opt path_searching hagen_msgs
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

include_directories(/usr/local/include/)
link_directories(/usr/local/lib/)

# TODO casadi not needed
add_executable(explorer src/explorer.cpp )
target_link_libraries(explorer ${catkin_LIBRARIES} casadi)
add_dependencies(explorer ${${PROJECT_NAME}_EXPORTED_TARGETS})
