cmake_minimum_required(VERSION 3.10)

project(dsp_global_mapping)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)


# Find the OctoMap package
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_filters
)

catkin_package()

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(global_mapping src/global_mapping.cpp)
target_link_libraries(global_mapping  ${PCL_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp ${OpenCV_LIBRARIES})
