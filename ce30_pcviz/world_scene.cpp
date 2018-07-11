#include "world_scene.h"
#include "grid_scene.h"
#include "sensor_model_scene.h"

namespace ce30_pcviz {
WorldScene::WorldScene(std::shared_ptr<pcl::visualization::PCLVisualizer> viz)
  : WorldSceneBase(viz) {
  grid_scene_.reset(new GridScene(viz));
  AddChild(grid_scene_);
  sensor_model_scene_.reset(new SensorModelScene(viz));
  AddChild(sensor_model_scene_);
}
} // namespace ce30_pcviz
