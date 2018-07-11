#include "world_scene_base.h"
#include <pcl/visualization/pcl_visualizer.h>

namespace ce30_pcviz {
WorldSceneBase::WorldSceneBase(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viz)
  : StaticScene(viz),
    change_default_viewpoint_on_show_(true),
    init_camera_parameters_on_show_(true)
{
}

WorldSceneBase::~WorldSceneBase() {}

void WorldSceneBase::ChangeDefaultViewPointOnShow(const bool &change) {
  change_default_viewpoint_on_show_ = change;
}

void WorldSceneBase::InitCameraParametersOnShow(const bool &init) {
  init_camera_parameters_on_show_ = init;
}

void WorldSceneBase::Show() {
  // Viz().addCoordinateSystem(1.0);
  if (init_camera_parameters_on_show_) {
    Viz().initCameraParameters();
  }
  if (change_default_viewpoint_on_show_) {
    Viz().setCameraPosition(-5, 0, 10, 10, 0, 0);
  }
}
} // namespace ce30_pcviz
