#ifndef SENSOR_MODEL_SCENE_H
#define SENSOR_MODEL_SCENE_H

#include "static_scene.h"

namespace ce30_pcviz {
class SensorModelScene : public StaticScene
{
public:
  SensorModelScene(std::shared_ptr<pcl::visualization::PCLVisualizer> viz);
  void Show() override;
};
}

#endif // SENSOR_MODEL_SCENE_H
