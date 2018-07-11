#ifndef GRID_SCENE_H
#define GRID_SCENE_H

#include "static_scene.h"
#include "export.h"

namespace ce30_pcviz {
class API GridScene : public StaticScene
{
public:
  GridScene(std::shared_ptr<pcl::visualization::PCLVisualizer> viz);
  virtual ~GridScene();
  void SetShowDefault(const bool& show_default);
  void SetParams(
      const int& width, const int& height, const float& size,
      const float& x, const float& y, const float& z = 0.0f);
  void Show() override;
protected:
  virtual void ShowDefault();
  bool show_default_;
  int width_;
  int height_;
  float size_;
  float x_;
  float y_;
  float z_;
};
} // namespace ce30_pcviz

#endif // GRID_SCENE_H
