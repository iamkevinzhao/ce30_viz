#ifndef GRID_SCENE_H
#define GRID_SCENE_H

#include "static_scene.h"
#include "export.h"

namespace ce30_pcviz {
class API GridScene : public StaticScene
{
public:
  GridScene(std::shared_ptr<pcl::visualization::PCLVisualizer> viz);
  void Show() override;
};
} // namespace ce30_pcviz

#endif // GRID_SCENE_H
