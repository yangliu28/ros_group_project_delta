cmake_minimum_required(VERSION 2.8.3)
project(ps9_hmi)

find_package(catkin_simple REQUIRED)

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_library(my_lib src/my_lib.cpp)   
cs_add_library(ps9_hmi_hand_detect src/ps9_hmi_hand_detect.cpp)

# Executables
# cs_add_executable(example src/example.cpp)
# target_link_library(example my_lib)

cs_add_executable(ps9_hmi_hand_detect_test src/ps9_hmi_hand_detect_test.cpp)

target_link_libraries(ps9_hmi_hand_detect_test ps9_hmi_hand_detect ${catkin_LIBRARIES})

cs_install()
cs_export()
