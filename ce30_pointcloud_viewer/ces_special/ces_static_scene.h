#ifndef CES_STATIC_SCENE_H
#define CES_STATIC_SCENE_H

#include <ce30_pcviz/scene.h>
#include <ce30_pcviz/world_scene.h>

class CESStaticScene : public ce30_pcviz::Scene
{
public:
  CESStaticScene();
  void SetShow(const bool& show);
  bool Showing();
  void Update() override;
private:
  void DrawScene();
  static std::string GetLineID(const int& id);
  bool showing_;
  bool last_showing_;
  std::unique_ptr<ce30_pcviz::WorldScene> world_scene_;
};

#endif // CES_STATIC_SCENE_H
