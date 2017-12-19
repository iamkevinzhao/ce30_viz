#include "world_scene.h"
#include "grid_scene.h"
#include "sensor_model_scene.h"

namespace ce30_pcviz {
WorldScene::WorldScene(std::shared_ptr<pcl::visualization::PCLVisualizer> viz)
  : StaticScene(viz),
    change_default_viewpoint_on_show_(true),
    init_camera_parameters_on_show_(true)
{
  grid_scene_.reset(new GridScene(viz));
  AddChild(grid_scene_);
  sensor_model_scene_.reset(new SensorModelScene(viz));
  AddChild(sensor_model_scene_);
}

void WorldScene::ChangeDefaultViewPointOnShow(const bool &change) {
  change_default_viewpoint_on_show_ = change;
}

void WorldScene::InitCameraParametersOnShow(const bool &init) {
  init_camera_parameters_on_show_ = init;
}

void WorldScene::Show() {
  // Viz().addCoordinateSystem(1.0);
  if (init_camera_parameters_on_show_) {
    Viz().initCameraParameters();
  }
  if (change_default_viewpoint_on_show_) {
    Viz().setCameraPosition(-5, 0, 10, 10, 0, 0);
  }
}
} // namespace ce30_pcviz
