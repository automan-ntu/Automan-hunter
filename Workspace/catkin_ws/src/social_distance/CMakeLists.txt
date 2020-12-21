cmake_minimum_required(VERSION 3.10.0)
project(social_distance)

find_package(darknet_ros_msgs REQUIRED)
find_package(darknet_ros REQUIRED)
find_package(PCL)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    pcl_conversions
    pcl_ros
    std_msgs
    cv_bridge
    darknet_ros_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES learning_joy
#  CATKIN_DEPENDS joy roscpp turtlesim
#  DEPENDS system_lib
)

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

include_directories(
include
${catkin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${OpenCV_INCLUDE_DIRS}
)

set(LIBS 
${PCL_LIBRARIES}
${catkin_LIBRARIES}
${OpenCV_LIBS}
)

add_executable(social_distance src/social_distance_node.cpp)
add_dependencies(social_distance ${catkin_EXPORTED_TARGETS})
target_link_libraries(social_distance ${LIBS})

if(MSVC)  
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zi")  
else()  
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")  
endif()
