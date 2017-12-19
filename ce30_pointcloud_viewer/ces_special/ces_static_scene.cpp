#include "ces_static_scene.h"
#include <iostream>
#include <ce30_pcviz/grid_scene.h>
#include <ce30_pcviz/sensor_model_scene.h>

using namespace std;
using namespace pcl;
using namespace pcl::visualization;
using namespace ce30_pcviz;

CESStaticScene::CESStaticScene()
  : Scene(nullptr), showing_(false),
    last_showing_(true) // ensure show on start
{
}

void CESStaticScene::SetShow(const bool &show) {
  showing_ = show;
}

bool CESStaticScene::Showing() {
  return showing_;
}

void CESStaticScene::Update() {
  if (last_showing_ != showing_) {
    if (showing_) {
      if (world_scene_) {
        world_scene_->Erase();
      }
      DrawScene();
    } else {
      Erase();
      world_scene_.reset(new WorldScene(VizPtr()));
      static bool first_world = true;
      if (first_world) {
        first_world = false;
      } else {
        world_scene_->InitCameraParametersOnShow(false);
        world_scene_->ChangeDefaultViewPointOnShow(false);
      }
      world_scene_->Update();
    }
  }
  last_showing_ = showing_;
}

void CESStaticScene::ChangeToDefaultViewPoint() {
  if (default_viewpoint_) {
    default_viewpoint_->Change(5.0f, -10.0f, 0.0f, 5.0f, 0.0f, 0.0f);
  }
}

void CESStaticScene::OnVisualizerLoaded(std::shared_ptr<PCLVisualizer> viz) {
  Scene::OnVisualizerLoaded();
  default_viewpoint_.reset(new StaticView(viz));
}

void CESStaticScene::DrawScene() {
  int id = -1;
  static vector<array<float, 6>> lines{
    {0.0f, -1.5f, -1.0f, 0.0f, 1.5f, -1.0f},
    {0.0f, -1.5f, -1.0f, 0.0f, -1.5f, 0.0f},
    {0.0f, -1.5f, 0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.5f},
    {0.0f, 0.0f, 1.5f, 0.0f, 1.5f, 1.5f},
    {0.0f, 1.5f, 1.5f, 0.0f, 1.5f, -1.0f},

    {0.0f, 1.5f, -1.0f, 9.0f, 1.5f, -1.0f},
    {9.0f, 1.5f, -1.0f, 9.0f, 1.5f, 1.5f},
    {9.0f, 1.5f, 1.5f, 0.0f, 1.5f, 1.5f},

    {9.0f, -1.5f, -1.0f, 9.0f, 1.5f, -1.0f},
    {9.0f, -1.5f, -1.0f, 9.0f, -1.5f, 0.0f},
    {9.0f, -1.5f, 0.0f, 9.0f, 0.0f, 0.0f},
    {9.0f, 0.0f, 0.0f, 9.0f, 0.0f, 1.5f},
    {9.0f, 0.0f, 1.5f, 9.0f, 1.5f, 1.5f},
    {9.0f, 1.5f, 1.5f, 9.0f, 1.5f, -1.0f},

    {0.0f, -1.5f, -1.0f, 9.0f, -1.5f, -1.0f}
  };
  double r = 0.0, g = 0.0, b = 1.0;
  float x = -0.5f, y = 0.0f, z = 0.0f;
  for (auto& line : lines) {
    Viz().addLine(
        PointXYZ(line[0] + x, line[1] + y, line[2] + z),
        PointXYZ(line[3] + x, line[4] + y, line[5] + z),
        r, g, b, GetLineID(++id));
  }
  vector<string> ids;
  ids.reserve(id);
  for (int i = 0; i <= id; ++i) {
    RegisterComponent(GetLineID(i));
  }

  shared_ptr<GridScene> grid_scene_(new GridScene(VizPtr()));
  AddChild(grid_scene_);
  // grid_scene_->Update();

  shared_ptr<SensorModelScene> sensor_model_scene_(
      new SensorModelScene(VizPtr()));
  sensor_model_scene_->SetTextDisplay(false);
  AddChild(sensor_model_scene_);
  sensor_model_scene_->Update();
}

string CESStaticScene::GetLineID(const int &id) {
  return "ces_scene_line_" + to_string(id);
}
