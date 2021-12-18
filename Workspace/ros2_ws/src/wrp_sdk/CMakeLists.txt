cmake_minimum_required(VERSION 3.5)
project(wrp_sdk)

# Find ament_cmake
find_package(ament_cmake REQUIRED)

# generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/devel)

## Set compiler to use c++ 14 features
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

## Optionally built modules: ON/OFF
set(BUILD_TESTS OFF)
set(BUILD_MONITOR ON)

# Disable monitor if ncurses library is not found
set(CURSES_NEED_NCURSES TRUE)
find_package(Curses QUIET)

if(BUILD_MONITOR AND NOT CURSES_FOUND)
    set(BUILD_MONITOR OFF)
    message(STATUS "Monitor app will not be built due to missing ncurses library, try: sudo apt install libncurses5-dev")
endif()

include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(${PROJECT_NAME} 
    src/async_serial.cpp
    src/async_can.cpp
    src/asyncio_utils.cpp
    # agilex mobile platforms
    src/platforms/mobile_base.cpp
    src/platforms/hunter_base.cpp
    src/platforms/hunter_can_parser.c
    src/platforms/scout_base.cpp
    src/platforms/scout_can_parser.c
    src/platforms/scout_uart_parser.c
)
target_compile_definitions(${PROJECT_NAME} PUBLIC "-DASIO_ENABLE_OLD_SERVICES -DASIO_HAS_POSIX_STREAM_DESCRIPTOR")
target_link_libraries(${PROJECT_NAME} pthread)
target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
# Add executables
if(BUILD_TESTS)
    add_subdirectory(tests)
    add_subdirectory(unit_tests)
endif()

## Export catkin targets
#catkin_package(
# INCLUDE_DIRS include
# LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS hunter_msgs roscpp 
#  DEPENDS system_lib
#)
ament_export_include_directories(include/wrp_sdk)
ament_export_libraries(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME}_export HAS_LIBRARY_TARGET)



## Use GNUInstallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)

## Add catkin install targets
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_export
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
  #DIRECTORY include/
  #DESTINATION include/
  FILES_MATCHING PATTERN "*"
  PATTERN ".git" EXCLUDE
)

ament_package()