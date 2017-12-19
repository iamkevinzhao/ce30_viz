#ifndef CES_STATIC_SCENE_H
#define CES_STATIC_SCENE_H

#include <ce30_pcviz/scene.h>
#include <ce30_pcviz/world_scene.h>
#include <ce30_pcviz/static_view.h>

class CESStaticScene : public ce30_pcviz::Scene
{
public:
  CESStaticScene();
  ~CESStaticScene();
  bool LoadParams();
  bool SaveParams();
  void SetShow(const bool& show);
  bool Showing();
  void Update() override;
  void ChangeToDefaultViewPoint();
  void UpdateOffsetDelta(const float& x, const float& y, const float& z);
protected:
  void OnVisualizerLoaded(
      std::shared_ptr<pcl::visualization::PCLVisualizer> viz) override;
private:
  std::string DefaultConfigFilePath();
  void DrawScene();
  static std::string GetLineID(const int& id);
  bool showing_;
  bool last_showing_;
  std::unique_ptr<ce30_pcviz::WorldScene> world_scene_;
  std::unique_ptr<ce30_pcviz::StaticView> default_viewpoint_;
  float offset_x_;
  float offset_y_;
  float offset_z_;
};

#endif // CES_STATIC_SCENE_H
