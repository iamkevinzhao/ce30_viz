configure_file(config.h.in "${CMAKE_CURRENT_BINARY_DIR}/config.h" @ONLY)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5Widgets)

add_library(
  ce30_pcviz
    SHARED
      ce30_pcviz.cpp
      ce30_pcviz.h
      config.h.in
      export.h
      scene.h
      scene.cpp
      world_scene.h
      world_scene.cpp
      grid_scene.h
      grid_scene.cpp
      static_scene.h
      static_scene.cpp
      grid_geometry.h
      grid_geometry.cpp
      sensor_model_scene.h
      sensor_model_scene.cpp
      operation_handler.h
      operation_handler.cpp
      static_view.h
      static_view.cpp
      cloud_scene.h
      cloud_scene.cpp
      cloud_cluster_scene.h
      cloud_cluster_scene.cpp
      gray_image.h
      gray_image.cpp
      grid_scene_x.h
      grid_scene_x.cpp
      world_scene_base.h
      world_scene_base.cpp
      world_scene_x.h
      world_scene_x.cpp
      control_panel_widget.h
      control_panel_widget.cpp
      control_panel_widget.ui
      control_panel_base.h
      control_panel_base.cpp
      helper_utils.h
      helper_utils.cpp
)

target_link_libraries(
  ce30_pcviz
    ${PCL_LIBRARIES}
    Qt5::Widgets
)

set_target_properties(ce30_pcviz PROPERTIES
  PUBLIC_HEADER "ce30_pcviz.h;${CMAKE_CURRENT_BINARY_DIR}/config.h")

install(TARGETS ce30_pcviz
  # IMPORTANT: Add the ce30_pcviz library to the "export-set"
  EXPORT ce30_pcvizTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/ce30_pcviz"
    COMPONENT dev)
