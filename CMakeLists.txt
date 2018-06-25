cmake_minimum_required(VERSION 2.8.3)
project(republish_topic)


find_package(catkin REQUIRED COMPONENTS
    pcl_ros
    roscpp
    std_msgs
)

include_directories(
  
  ${catkin_INCLUDE_DIRS}
  include
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    pcl_ros
    roscpp
    std_msgs
)

SET(CMAKE_BUILD_TYPE "Debug")  
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O1 -Wall -g -ggdb")  
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )
# opencv 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# eigen 
include_directories( "/usr/include/eigen3/" )

# pcl 
find_package( PCL 1.7 REQUIRED )
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )



add_executable( repub_pointcloud src/repub_pointcloud.cpp )
target_link_libraries( repub_pointcloud ${OpenCV_LIBS} ${PCL_LIBRARIES} ${catkin_LIBRARIES} )

