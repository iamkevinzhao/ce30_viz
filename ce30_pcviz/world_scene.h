#ifndef WORLD_SCENE_H
#define WORLD_SCENE_H

#include "static_scene.h"
#include "export.h"

namespace ce30_pcviz {
class API WorldScene : public StaticScene
{
public:
  WorldScene(std::shared_ptr<pcl::visualization::PCLVisualizer> viz);
  void Show() override;
private:
  std::shared_ptr<Scene> grid_scene_;
  std::shared_ptr<Scene> sensor_model_scene_;
};
} // namespace ce30_pcviz

#endif // WORLD_SCENE_H
