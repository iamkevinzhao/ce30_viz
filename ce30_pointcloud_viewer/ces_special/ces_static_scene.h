#ifndef CES_STATIC_SCENE_H
#define CES_STATIC_SCENE_H

#include <ce30_pcviz/scene.h>

class CESStaticScene : public ce30_pcviz::Scene
{
public:
  CESStaticScene();
  void SetShow(const bool& show);
  bool Showing();
  void Update() override;
private:
  bool showing_;
};

#endif // CES_STATIC_SCENE_H
