cmake_minimum_required(VERSION 2.8)
project(ce30_viz C CXX)

set(CMAKE_CXX_STANDARD 11)

set(CE30_VIZ_MAJOR_VERSION 1)
set(CE30_VIZ_MINOR_VERSION 10)
set(CE30_VIZ_PATCH_VERSION 0)
set(CE30_VIZ_VERSION
  ${CE30_VIZ_MAJOR_VERSION}.${CE30_VIZ_MINOR_VERSION}.${CE30_VIZ_PATCH_VERSION})

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH
  "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/cmake/ce30_viz)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
  "Installation directory for CMake files")

# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

# set up include-directories
include_directories(
  "${PROJECT_SOURCE_DIR}"   # to find ce30_pcviz/ce30_pcviz.h
  "${PROJECT_BINARY_DIR}")  # to find ce30_pcviz/config.h

find_package(PCL 1.7 REQUIRED)
# find_package(ce30_driver 1.2.1 REQUIRED)
find_package(ce30_driver 1.3.0 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add sub-directories
if (${CE30_VISUALIZER})
  add_subdirectory(ce30_drivers)
endif()
add_subdirectory(ce30_pcviz)
add_subdirectory(ce30_pointcloud_viewer)
if (${CE30_VISUALIZER})
  add_subdirectory(ce30_visualizer)
endif()
# add_subdirectory(ce30_depthimage_viewer)

# The interesting stuff goes here
# ===============================

# Add all targets to the build-tree export set
SET(
  EXPORT_TARGETS
    # ce30_drivers
    ce30_pcviz
    ce30_pointcloud_viewer
)

if (${CE30_VISUALIZER})
  SET(EXPORT_TARGETS "${EXPORT_TARGETS};ce30_drivers")
endif()

if (${CE30_VISUALIZER})
  SET(
    EXPORT_TARGETS
      ${EXPORT_TARGETS}
      ce30_visualizer
  )
endif()

export(
  TARGETS
    ${EXPORT_TARGETS}
  FILE "${PROJECT_BINARY_DIR}/ce30_vizTargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE ce30_viz)

# Create the ce30_vizConfig.cmake and ce30_vizConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
   "${INSTALL_INCLUDE_DIR}")
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(ce30_vizConfig.cmake.in
  "${PROJECT_BINARY_DIR}/ce30_vizConfig.cmake" @ONLY)
# ... for the install tree
set(CONF_INCLUDE_DIRS "\${CE30_VIZ_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(ce30_vizConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/ce30_vizConfig.cmake" @ONLY)
# ... for both
configure_file(ce30_vizConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/ce30_vizConfigVersion.cmake" @ONLY)

# Install the ce30_vizConfig.cmake and ce30_vizConfigVersion.cmake
install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/ce30_vizConfig.cmake"
  "${PROJECT_BINARY_DIR}/ce30_vizConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# Install the export set for use with the install-tree
install(EXPORT ce30_vizTargets DESTINATION
  "${INSTALL_CMAKE_DIR}" COMPONENT dev)
