# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_smart_cane_msgs_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED smart_cane_msgs_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(smart_cane_msgs_FOUND FALSE)
  elseif(NOT smart_cane_msgs_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(smart_cane_msgs_FOUND FALSE)
  endif()
  return()
endif()
set(_smart_cane_msgs_CONFIG_INCLUDED TRUE)

# output package information
if(NOT smart_cane_msgs_FIND_QUIETLY)
  message(STATUS "Found smart_cane_msgs: 0.0.0 (${smart_cane_msgs_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'smart_cane_msgs' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${smart_cane_msgs_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(smart_cane_msgs_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${smart_cane_msgs_DIR}/${_extra}")
endforeach()
