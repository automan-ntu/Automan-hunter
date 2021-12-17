cmake_minimum_required(VERSION 3.5)
project(hunter_base)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(hunter_msgs REQUIRED)
find_package(wrp_sdk REQUIRED)
#add_subdirectory(ascent)

#ament_export_include_directories(include)
#ament_export_libraries(hunter_messenger)

include_directories(include)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include/"
                    "${CMAKE_CURRENT_SOURCE_DIR}/ascent/include"
                    )

add_library(hunter_messenger STATIC 
    src/hunter_messenger.cpp 
    src/bicycle_model.cpp
    )  

target_link_libraries(hunter_messenger 
                      hunter_msgs
                      nav_msgs 
                      rclcpp 
                      std_msgs
                      )

add_executable(hunter_base_node src/hunter_base_node.cpp)

target_link_libraries(hunter_base_node 
                      hunter_messenger
                      wrp_sdk 
                      rclcpp 
                      std_msgs
                      )

add_executable(hunter_status_node src/hunter_status_node.cpp)
target_link_libraries(hunter_status_node 
                      hunter_messenger 
                      wrp_sdk 
                      rclcpp 
                      std_msgs
                      )

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

ament_package()