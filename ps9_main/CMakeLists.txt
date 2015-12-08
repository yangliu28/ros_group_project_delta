cmake_minimum_required(VERSION 2.8.3)
project(ps9_main)

# find_package(catkin_simple REQUIRED)

find_package(catkin_simple REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  tf
  tf2
  cwru_pcl_utils
  ps9_pcl
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})


find_package(Boost REQUIRED COMPONENTS system thread)


find_package(ps9_pcl REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ps9_pcl_INCLUDE_DIRS}
)

find_package(ps9_arm_motion REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ps9_arm_motion_INCLUDE_DIRS}
)

find_package(ps9_gripper_control REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ps9_gripper_control_INCLUDE_DIRS}
)

find_package(ps9_hmi REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ps9_hmi_INCLUDE_DIRS}
)

#catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
#cs_add_library(ps9_pcl src/block_detection.cpp)

# Executables
add_executable(ps9_main src/ps9_main.cpp)
add_executable(ps9_main_debug src/ps9_main_debug.cpp)
# this is weird
target_link_libraries(ps9_main ps9_pcl ps9_arm_motion_commander_lib ps9_gripper_control ps9_hmi_hand_detect ${catkin_LIBRARIES})
target_link_libraries(ps9_main_debug ps9_pcl ps9_arm_motion_commander_lib ps9_gripper_control ps9_hmi_hand_detect ${catkin_LIBRARIES})

# cs_install()
# cs_export()
