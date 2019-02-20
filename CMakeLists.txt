cmake_minimum_required(VERSION 2.8)
project(FisheyeDev CXX)

set(FISHEYEDEV_MAJOR_VERSION 0)
set(FISHEYEDEV_MINOR_VERSION 1)
set(FISHEYEDEV_PATCH_VERSION 0)
set(FISHEYEDEV_VERSION
  ${FISHEYEDEV_MAJOR_VERSION}.${FISHEYEDEV_MINOR_VERSION}.${FISHEYEDEV_PATCH_VERSION})

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH
  "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/CMake/FisheyeDev)
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
  "${PROJECT_SOURCE_DIR}"   # to find fisheye_common/fisheye_common.h
  "${PROJECT_BINARY_DIR}")  # to find fisheye_common/config.h

# Add sub-directories
add_subdirectory(fisheye_common)
add_subdirectory(fisheye_undistorted)

# The interesting stuff goes here
# ===============================

# Add all targets to the build-tree export set
export(TARGETS fisheye_common fisheye_undistorted
  FILE "${PROJECT_BINARY_DIR}/FisheyeDevTargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE FisheyeDev)

# Create the FisheyeDevConfig.cmake and FisheyeDevConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
   "${INSTALL_INCLUDE_DIR}")
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(FisheyeDevConfig.cmake.in
  "${PROJECT_BINARY_DIR}/FisheyeDevConfig.cmake" @ONLY)
# ... for the install tree
set(CONF_INCLUDE_DIRS "\${FISHEYEDEV_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(FisheyeDevConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/FisheyeDevConfig.cmake" @ONLY)
# ... for both
configure_file(FisheyeDevConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/FisheyeDevConfigVersion.cmake" @ONLY)

# Install the FisheyeDevConfig.cmake and FisheyeDevConfigVersion.cmake
install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/FisheyeDevConfig.cmake"
  "${PROJECT_BINARY_DIR}/FisheyeDevConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# Install the export set for use with the install-tree
install(EXPORT FisheyeDevTargets DESTINATION
  "${INSTALL_CMAKE_DIR}" COMPONENT dev)
