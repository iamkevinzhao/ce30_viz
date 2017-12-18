#ifndef SENSOR_MODEL_SCENE_H
#define SENSOR_MODEL_SCENE_H

#include "static_scene.h"

namespace ce30_pcviz {
class API SensorModelScene : public StaticScene
{
public:
  SensorModelScene(std::shared_ptr<pcl::visualization::PCLVisualizer> viz);
  void SetTextDisplay(const bool& display);
  void Show() override;
private:
  bool display_text_;
};
}

#endif // SENSOR_MODEL_SCENE_H
