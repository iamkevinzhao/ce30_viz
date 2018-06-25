configure_file(
  config.h.in
  "${CMAKE_CURRENT_BINARY_DIR}/config.h" @ONLY)

if ("${Boost_CUSTOMIZED_ROOT}" STREQUAL "")
  message("Using default Boost root directory")
else()
  message("Using customized Boost library directory")
  set(BOOST_ROOT "${Boost_CUSTOMIZED_ROOT}")
endif()

find_package(Boost COMPONENTS system REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

IF(MSVC)
    ADD_DEFINITIONS("/EHsc")
ENDIF(MSVC)

if ("${Boost_CUSTOMIZED_LIBRARYDIR}" STREQUAL "")
  message("Using default Boost library directory")
else()
  message("Using customized Boost library directory")
  link_directories("${Boost_CUSTOMIZED_LIBRARYDIR}")
endif()

add_library(
  ce30_drivers
    SHARED
      model_d_utils.h
      model_d_utils.cpp
      config.h.in
      udp_socket.cpp
      udp_socket.h
      timed_udp_socket.cpp
      timed_udp_socket.h
      model_d_packets.h
      model_d_packets.cpp
      model_d_udp_server.h
      model_d_udp_server.cpp
      model_x_packets.cpp
      model_x_packets.h
      data_types.h
      data_types.cpp
      ce30_drivers_export.h
      ce30_d_driver.h
      ce30_x_driver.h
      common_utils.h
      common_utils.cpp
      packets.h
      packets.cpp
)

target_link_libraries(
  ce30_drivers
    ${Boost_LIBRARIES}
)

set(
  PUBLIC_HEADERS

    ${CMAKE_CURRENT_BINARY_DIR}/config.h
)

set_target_properties(ce30_drivers PROPERTIES
  PUBLIC_HEADER "${PUBLIC_HEADERS}")

install(TARGETS ce30_drivers
  # IMPORTANT: Add the ce30_drivers library to the "export-set"
  EXPORT ce30_driversTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/ce30_drivers"
    COMPONENT dev)
