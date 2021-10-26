cmake_minimum_required(VERSION 3.10.0)
project(dense_scene)

find_package(PCL)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    pcl_conversions
    pcl_ros
    std_msgs
    cv_bridge
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

add_executable(dense_scene src/dense_scene.cpp)
#add_dependencies(dense_scene ${catkin_EXPORTED_TARGETS})
target_link_libraries(dense_scene ${LIBS})

if(MSVC)  
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zi")  
else()  
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")  
endif()
