#include "world_scene.h"

namespace ce30_pcviz {
WorldScene::WorldScene(std::shared_ptr<pcl::visualization::PCLVisualizer> viz)
  : Scene(viz), shown_(false)
{
}

void WorldScene::Update() {
  if (!shown_) {
    Show();
    shown_ = true;
  }
}

void WorldScene::Show() {
  Viz().addCoordinateSystem(1.0);
  Viz().initCameraParameters();
  Viz().setCameraPosition(-5, 0, 10, 10, 0, 0);
}
} // namespace ce30_pcviz
