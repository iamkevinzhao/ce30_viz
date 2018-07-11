#ifndef WORLD_SCENE_X_H
#define WORLD_SCENE_X_H

#include "world_scene_base.h"

namespace ce30_pcviz {
class API WorldSceneX : public WorldSceneBase
{
 public:
  WorldSceneX(std::shared_ptr<pcl::visualization::PCLVisualizer> viz);
 private:
  std::shared_ptr<Scene> grid_scene_;
  std::shared_ptr<Scene> sensor_model_scene_;
};
} // namespace ce30_pcviz

#endif // WORLD_SCENE_X_H
