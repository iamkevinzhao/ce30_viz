#ifndef WORLD_SCENE_H
#define WORLD_SCENE_H

#include "scene.h"

namespace ce30_pcviz {
class WorldScene : public Scene
{
public:
  WorldScene(std::shared_ptr<pcl::visualization::PCLVisualizer> viz);
  void Update() override;
private:
  void Show();
  bool shown_;
};
} // namespace ce30_pcviz

#endif // WORLD_SCENE_H
