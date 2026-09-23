# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rog_map_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rog_map_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rog_map_FOUND FALSE)
  elseif(NOT rog_map_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rog_map_FOUND FALSE)
  endif()
  return()
endif()
set(_rog_map_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rog_map_FIND_QUIETLY)
  message(STATUS "Found rog_map: 0.0.0 (${rog_map_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rog_map' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT rog_map_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rog_map_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rog_map_DIR}/${_extra}")
endforeach()
