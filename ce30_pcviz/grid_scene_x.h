#ifndef GRID_SCENE_X_H
#define GRID_SCENE_X_H

#include "grid_scene.h"

namespace ce30_pcviz {
class API GridSceneX : public GridScene {
 public:
  GridSceneX(std::shared_ptr<pcl::visualization::PCLVisualizer> viz);
 protected:
  void ShowDefault() override;
};
} // namespace ce30_pcviz

#endif // GRID_SCENE_X_H
