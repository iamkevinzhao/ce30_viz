#ifndef WORLD_SCENE_BASE_H
#define WORLD_SCENE_BASE_H

#include "static_scene.h"
#include "export.h"

namespace ce30_pcviz {
class API WorldSceneBase : public StaticScene
{
public:
  WorldSceneBase(std::shared_ptr<pcl::visualization::PCLVisualizer> viz);
  virtual ~WorldSceneBase();
  void Show() override;
  void ChangeDefaultViewPointOnShow(const bool& change);
  void InitCameraParametersOnShow(const bool& init);
private:
  bool change_default_viewpoint_on_show_;
  bool init_camera_parameters_on_show_;
};
} // namespace ce30_pcviz

#endif // WORLD_SCENE_BASE_H
