set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5Widgets)
find_package(Qt5Core)

if(USE_FAKE_POINTCLOUD)
  add_definitions(-DFAKE_POINTCLOUD)
  set(
    SRC_FAKE_POINTCLOUD
      fake_point_cloud_viewer.h
      fake_point_cloud_viewer.cpp
  )
endif()

if(ON_DEVEL)
  add_definitions(-DON_DEVEL)
endif()

if(SUPPORT_GRAY_OUTPUT_MODE)
  add_definitions(-DSUPPORT_GRAY_OUTPUT_MODE)
endif()

if (SUPPORT_CHANNEL_TYPE_FEATURE)
  add_definitions(-DSUPPORT_CHANNEL_TYPE_FEATURE)
endif()

if(CES_SPECIAL)
  add_definitions(-DCES_SPECIAL)
  set(
    SRC_CES_SPECIAL
      ces_special/ces_special.h
      ces_special/ces_special.cpp
      ces_special/ces_static_scene.h
      ces_special/ces_static_scene.cpp
  )
endif()

if(USE_FEATURE_FILTER)
  add_definitions(-DUSE_FEATURE_FILTER)
endif()

add_executable(
  ce30_pointcloud_viewer
    main.cpp
    point_cloud_viewer.cpp
    point_cloud_viewer.h
    grey_image_window.cpp
    grey_image_window.h
    grey_image_window.ui
    grey_image.cpp
    grey_image.h
    ${SRC_FAKE_POINTCLOUD}
    ${SRC_CES_SPECIAL}
    exit_code.h
)

target_link_libraries(
  ce30_pointcloud_viewer
    ce30_pcviz
    ${CE30_DRIVER_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
)

install(TARGETS ce30_pointcloud_viewer
  # IMPORTANT: Add the ce30_pointcloud_viewer executable to the "export-set"
  EXPORT ce30_vizTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
