#include "ces_static_scene.h"
#include <iostream>
#include <ce30_pcviz/grid_scene.h>
#include <ce30_pcviz/sensor_model_scene.h>
#include <fstream>

using namespace std;
using namespace pcl;
using namespace pcl::visualization;
using namespace ce30_pcviz;

CESStaticScene::CESStaticScene()
  : Scene(nullptr), showing_(false),
    last_showing_(true), // ensure show on start
    offset_x_(-0.5f),
    offset_y_(0.0f),
    offset_z_(0.0f)
{
  LoadParams();
}

CESStaticScene::~CESStaticScene() {
  SaveParams();
}

bool CESStaticScene::LoadParams() {
  ifstream is(DefaultConfigFilePath());
  if (!is.is_open()) {
    return false;
  }
  is >> offset_x_;
  is >> offset_y_;
  is >> offset_z_;
  is.close();
  return true;
}

bool CESStaticScene::SaveParams() {
  ofstream os(DefaultConfigFilePath());
  if (!os.is_open()) {
    return false;
  }
  os << offset_x_ << " " << offset_y_ << " " << offset_z_ << endl;
  return true;
}

string CESStaticScene::DefaultConfigFilePath() {
  return "settings_ces.txt";
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

void CESStaticScene::UpdateOffsetDelta(
    const float &x, const float &y, const float &z) {
  offset_x_ += x;
  offset_y_ += y;
  offset_z_ += z;
  Erase();
  DrawScene();
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
  for (auto& line : lines) {
    Viz().addLine(
        PointXYZ(line[0] + offset_x_, line[1] + offset_y_, line[2] + offset_z_),
        PointXYZ(line[3] + offset_x_, line[4] + offset_y_, line[5] + offset_z_),
        r, g, b, GetLineID(++id));
  }
  vector<string> ids;
  ids.reserve(id);
  for (int i = 0; i <= id; ++i) {
    RegisterComponent(GetLineID(i));
  }

  shared_ptr<GridScene> grid_scene_(new GridScene(VizPtr()));
  grid_scene_->SetParams(
      12, 30, 0.5f, 12.0f + offset_x_, 3.0f + offset_y_, -1.1f + offset_z_);
  AddChild(grid_scene_);
  grid_scene_->Update();

  shared_ptr<SensorModelScene> sensor_model_scene_(
      new SensorModelScene(VizPtr()));
  sensor_model_scene_->SetTextDisplay(false);
  AddChild(sensor_model_scene_);
  sensor_model_scene_->Update();
}

string CESStaticScene::GetLineID(const int &id) {
  return "ces_scene_line_" + to_string(id);
}
