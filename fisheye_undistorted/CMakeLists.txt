add_executable(fisheye_undistorted fisheye_undistorted.cpp)

target_link_libraries(fisheye_undistorted fisheye_common)

install(TARGETS fisheye_undistorted
  # IMPORTANT: Add the fisheye_undistorted executable to the "export-set"
  EXPORT FisheyeDevTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
