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
  void ChangeDefaultViewPointOnShow(const bool& change);
  void InitCameraParametersOnShow(const bool& init);
private:
  std::shared_ptr<Scene> grid_scene_;
  std::shared_ptr<Scene> sensor_model_scene_;
  bool change_default_viewpoint_on_show_;
  bool init_camera_parameters_on_show_;
};
} // namespace ce30_pcviz

#endif // WORLD_SCENE_H
