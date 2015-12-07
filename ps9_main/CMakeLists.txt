cmake_minimum_required(VERSION 2.8.3)
project(ps9_main)

find_package(catkin_simple REQUIRED)

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_library(my_lib src/my_lib.cpp)   

# Executables
cs_add_executable(ps9_main src/ps9_main.cpp)
# this is weird
target_link_libraries(ps9_main block_detection ${catkin_LIBRARIES})

cs_install()
cs_export()
