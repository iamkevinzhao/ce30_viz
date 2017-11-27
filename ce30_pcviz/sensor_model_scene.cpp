#include "sensor_model_scene.h"

using namespace std;
using namespace pcl::visualization;

namespace ce30_pcviz {
SensorModelScene::SensorModelScene(shared_ptr<PCLVisualizer> viz)
  : StaticScene(viz)
{

}

void SensorModelScene::Show() {
  Viz().addCube(-0.1f, 0.1f, -0.15f, 0.15f, -0.1f, 0.1f);
}
}
