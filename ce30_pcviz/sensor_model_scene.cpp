#include "sensor_model_scene.h"
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl::visualization;

namespace ce30_pcviz {
SensorModelScene::SensorModelScene(shared_ptr<PCLVisualizer> viz)
  : StaticScene(viz), display_text_(true)
{

}

void SensorModelScene::Show() {
  string camera_mode_id = "camera";
  Viz().addCube(
      -0.1f, 0.1f, -0.15f, 0.15f, -0.1f, 0.1f,
      1.0, 1.0, 1.0, camera_mode_id);
  RegisterComponent(camera_mode_id);
  string ce30_text_id = "ce30_text";
  if (display_text_) {
    Viz().addText3D(
        "CE30", pcl::PointXYZ(-1.0f, 0.0f, 0.0f), 0.5f,
        1.0, 1.0, 1.0, ce30_text_id);
    RegisterComponent(ce30_text_id);
  }
}

void SensorModelScene::SetTextDisplay(const bool &display) {
  display_text_ = display;
}
}
