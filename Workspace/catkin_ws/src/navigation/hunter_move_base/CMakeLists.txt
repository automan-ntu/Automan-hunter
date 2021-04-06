cmake_minimum_required(VERSION 3.0.2)
project(hunter_move_base)

find_package(catkin REQUIRED
    COMPONENTS
        actionlib
        base_local_planner
        clear_costmap_recovery
        cmake_modules
        costmap_2d
        dynamic_reconfigure
        geometry_msgs
        message_generation
        move_base_msgs
        nav_core
        nav_msgs
        navfn
        pluginlib
        roscpp
        rospy
        rotate_recovery
        std_srvs
        tf2_geometry_msgs
        tf2_ros
)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/MoveBase.cfg
)

catkin_package(
    CATKIN_DEPENDS
        dynamic_reconfigure
        geometry_msgs
        move_base_msgs
        nav_msgs
        roscpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# hunter_move_base
add_library(hunter_move_base
  src/hunter_move_base.cpp
)
target_link_libraries(hunter_move_base
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )
add_dependencies(hunter_move_base ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(hunter_move_base_node
  src/hunter_move_base_node.cpp
)
add_dependencies(hunter_move_base_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hunter_move_base_node hunter_move_base)
set_target_properties(hunter_move_base_node PROPERTIES OUTPUT_NAME hunter_move_base)

install(
    TARGETS
        hunter_move_base_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    TARGETS
        hunter_move_base
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
