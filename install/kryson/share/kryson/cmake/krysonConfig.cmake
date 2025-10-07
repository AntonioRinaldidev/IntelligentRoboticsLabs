# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_kryson_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED kryson_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(kryson_FOUND FALSE)
  elseif(NOT kryson_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(kryson_FOUND FALSE)
  endif()
  return()
endif()
set(_kryson_CONFIG_INCLUDED TRUE)

# output package information
if(NOT kryson_FIND_QUIETLY)
  message(STATUS "Found kryson: 0.0.0 (${kryson_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'kryson' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT kryson_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(kryson_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${kryson_DIR}/${_extra}")
endforeach()
