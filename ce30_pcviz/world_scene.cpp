#include "world_scene.h"
#include "grid_scene.h"
#include "sensor_model_scene.h"

namespace ce30_pcviz {
WorldScene::WorldScene(std::shared_ptr<pcl::visualization::PCLVisualizer> viz)
  : StaticScene(viz)
{
  grid_scene_.reset(new GridScene(viz));
  AddChild(grid_scene_);
  sensor_model_scene_.reset(new SensorModelScene(viz));
  AddChild(sensor_model_scene_);
}

void WorldScene::Show() {
//  Viz().addCoordinateSystem(1.0);
  Viz().initCameraParameters();
  Viz().setCameraPosition(-5, 0, 10, 10, 0, 0);
}
} // namespace ce30_pcviz
