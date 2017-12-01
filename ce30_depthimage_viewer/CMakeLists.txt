# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

set(CMAKE_AUTOUIC ON)

# Find the QtWidgets library
find_package(Qt5Widgets)

# Tell CMake to create the helloworld executable
add_executable(
  ce30_depthimage_viewer
    main.cpp
    main_window.h
    main_window.cpp
    main_window.ui
    window_utils.h
    window_utils.cpp
    image_manager.h
    image_manager.cpp
    config.h
    config.cpp
    gray_image.h
    gray_image.cpp
    main_window_image_control.h
    main_window_image_control.cpp
    image_server.h
    image_server.cpp
    main_window_image_service.h
    main_window_image_service.cpp
    settings_dialog.h
    settings_dialog.cpp
    settings_dialog.ui
    utilities.h
    utilities.cpp
    device_manager.h
    device_manager.cpp
    main_window_point_cloud_service.h
    main_window_point_cloud_service.cpp
    point_cloud_viewer.h
    point_cloud_viewer.cpp
)

target_link_libraries(
  ce30_depthimage_viewer
    ${CE30_DRIVER_LIBRARIES}
    Qt5::Widgets
)

if(UNIX)
  target_link_libraries(
    ce30_depthimage_viewer
      pthread
      -lboost_system
  )
endif()

install(TARGETS ce30_depthimage_viewer
  # IMPORTANT: Add the ce30_depthimage_viewer executable to the "export-set"
  EXPORT ce30_vizTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
