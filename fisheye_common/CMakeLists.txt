configure_file(
  config.h.in
  "${CMAKE_CURRENT_BINARY_DIR}/config.h" @ONLY)

add_library(
  fisheye_common
    SHARED
      fisheye_common.cpp
      fisheye_common.h
      undistort.cpp
      undistort.h
      config.h.in
)

target_link_libraries(
  fisheye_common
    ${OpenCV_LIBS}
)

set(PUBLIC_HEADERS
  fisheye_common.h
  undistort.h
  ${CMAKE_CURRENT_BINARY_DIR}/config.h
)

set_target_properties(fisheye_common PROPERTIES
  PUBLIC_HEADER "${PUBLIC_HEADERS}")

install(TARGETS fisheye_common
  # IMPORTANT: Add the fisheye_common library to the "export-set"
  EXPORT FisheyeDevTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/fisheye_common"
    COMPONENT dev)
