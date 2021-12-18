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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(hunter_msgs REQUIRED)
find_package(wrp_sdk REQUIRED )

#message("wrp_sdk's path: " ${wrp_sdk_INCLUDE_DIRS})
#add_library(hunter_messenger  
#    src/hunter_messenger.cpp 
#    src/bicycle_model.cpp
#  )
#target_include_directories(hunter_messenger PUBLIC
#    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/>
#    $<INSTALL_INTERFACE:include/>
#    PRIVATE ${wrp_sdk_INCLUDE_DIRS}
#    ${PROJECT_SOURCE_DIR}/ascent/include
#  )
#ament_target_dependencies(hunter_messenger
#                      wrp_sdk
#                      hunter_msgs
#                      nav_msgs 
#                      rclcpp 
#                      std_msgs
#                      geometry_msgs
#                      )

add_executable(hunter_base_node 
              src/hunter_base_node.cpp
              src/hunter_base_ros.cpp
              src/hunter_messenger.cpp 
              src/bicycle_model.cpp
              )
#target_link_libraries(hunter_base_node wrp_sdk)

target_include_directories(hunter_base_node PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>
    PRIVATE ${wrp_sdk_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/ascent/include
  )

ament_target_dependencies(hunter_base_node
                      hunter_msgs
                      wrp_sdk 
                      rclcpp 
                      std_msgs
                      geometry_msgs
                      nav_msgs
                      tf2 
                      tf2_ros
                      )

#add_executable(hunter_status_node src/hunter_status_node.cpp)
#ament_target_dependencies(hunter_status_node 
#                      wrp_sdk 
#                      rclcpp 
#                      std_msgs
#                      )
install(TARGETS hunter_base_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

ament_package()