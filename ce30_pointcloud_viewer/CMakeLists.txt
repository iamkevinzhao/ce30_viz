add_executable(
  ce30_pointcloud_viewer
  main.cpp)

target_link_libraries(ce30_pointcloud_viewer ce30_pcviz)

install(TARGETS ce30_pointcloud_viewer
  # IMPORTANT: Add the ce30_pointcloud_viewer executable to the "export-set"
  EXPORT ce30_vizTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
