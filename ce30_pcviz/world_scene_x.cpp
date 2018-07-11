#include "world_scene_x.h"
#include "grid_scene_x.h"
#include "sensor_model_scene.h"

namespace ce30_pcviz {
WorldSceneX::WorldSceneX(std::shared_ptr<pcl::visualization::PCLVisualizer> viz)
  : WorldSceneBase(viz)
{
  grid_scene_.reset(new GridSceneX(viz));
  AddChild(grid_scene_);
  auto sensor_mode_scene = new SensorModelScene(viz);
  sensor_mode_scene->SetTextDisplay(false);
  sensor_model_scene_.reset(sensor_mode_scene);
  AddChild(sensor_model_scene_);
}
} // namespace ce30_pcviz
