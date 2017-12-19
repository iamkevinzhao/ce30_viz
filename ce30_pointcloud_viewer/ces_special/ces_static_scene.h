#ifndef CES_STATIC_SCENE_H
#define CES_STATIC_SCENE_H

#include <ce30_pcviz/scene.h>
#include <ce30_pcviz/world_scene.h>
#include <ce30_pcviz/static_view.h>

class CESStaticScene : public ce30_pcviz::Scene
{
public:
  CESStaticScene();
  void SetShow(const bool& show);
  bool Showing();
  void Update() override;
  void ChangeToDefaultViewPoint();
protected:
  void OnVisualizerLoaded(
      std::shared_ptr<pcl::visualization::PCLVisualizer> viz) override;
private:
  void DrawScene();
  static std::string GetLineID(const int& id);
  bool showing_;
  bool last_showing_;
  std::unique_ptr<ce30_pcviz::WorldScene> world_scene_;
  std::unique_ptr<ce30_pcviz::StaticView> default_viewpoint_;
};

#endif // CES_STATIC_SCENE_H
