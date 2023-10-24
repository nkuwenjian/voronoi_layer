cmake_minimum_required(VERSION 3.0.2)
project(voronoi_layer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")

# set(CMAKE_CXX_CLANG_TIDY
#   clang-tidy;
#   -header-filter=${PROJECT_SOURCE_DIR}/voronoi_layer;
#   -warnings-as-errors=*;
# )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  nav_msgs
)

# Glog
find_package(glog REQUIRED)

catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR} third_party
  LIBRARIES voronoi_layer
  CATKIN_DEPENDS
    costmap_2d
    nav_msgs
)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
  third_party
)

add_library(voronoi_layer voronoi_layer/voronoi_layer.cc third_party/dynamicvoronoi/dynamicvoronoi.cpp)
target_link_libraries(voronoi_layer ${catkin_LIBRARIES} glog::glog)
