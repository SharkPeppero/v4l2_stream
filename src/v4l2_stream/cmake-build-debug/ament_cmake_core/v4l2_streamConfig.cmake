# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_v4l2_stream_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED v4l2_stream_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(v4l2_stream_FOUND FALSE)
  elseif(NOT v4l2_stream_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(v4l2_stream_FOUND FALSE)
  endif()
  return()
endif()
set(_v4l2_stream_CONFIG_INCLUDED TRUE)

# output package information
if(NOT v4l2_stream_FIND_QUIETLY)
  message(STATUS "Found v4l2_stream: 0.0.0 (${v4l2_stream_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'v4l2_stream' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${v4l2_stream_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(v4l2_stream_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${v4l2_stream_DIR}/${_extra}")
endforeach()
