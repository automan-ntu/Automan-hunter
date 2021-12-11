# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_wrp_sdk_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED wrp_sdk_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(wrp_sdk_FOUND FALSE)
  elseif(NOT wrp_sdk_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(wrp_sdk_FOUND FALSE)
  endif()
  return()
endif()
set(_wrp_sdk_CONFIG_INCLUDED TRUE)

# output package information
if(NOT wrp_sdk_FIND_QUIETLY)
  message(STATUS "Found wrp_sdk: 0.3.3 (${wrp_sdk_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'wrp_sdk' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${wrp_sdk_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(wrp_sdk_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake")
foreach(_extra ${_extras})
  include("${wrp_sdk_DIR}/${_extra}")
endforeach()
