#ifndef STATIC_SCENE_H
#define STATIC_SCENE_H

#include "scene.h"
#include "export.h"

namespace ce30_pcviz {
class API StaticScene : public Scene
{
public:
  StaticScene(std::shared_ptr<pcl::visualization::PCLVisualizer> viz);
  virtual ~StaticScene();
  void Update() override;
protected:
  virtual void Show();
private:
  bool shown_;
};
}
#endif // STATIC_SCENE_H
