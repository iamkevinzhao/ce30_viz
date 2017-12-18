#include "ces_static_scene.h"
#include <iostream>

using namespace std;
using namespace pcl;
using namespace pcl::visualization;

CESStaticScene::CESStaticScene()
  : Scene(nullptr), showing_(false), last_showing_(false)
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
      scene_ids_ = DrawScene(Viz());
    } else {
      EraseScene(Viz(), scene_ids_);
    }
  }
  last_showing_ = showing_;
}

vector<string> CESStaticScene::DrawScene(PCLVisualizer& viz) {
  int id = -1;
  viz.addLine(
      PointXYZ(0.0f, -1.5f, -1.0f),
      PointXYZ(0.0f, 1.5f, -1.0f), GetLineID(++id));
  vector<string> ids;
  ids.reserve(id);
  for (int i = 0; i <= id; ++i) {
    ids.push_back(GetLineID(i));
  }
  return ids;
}

void CESStaticScene::EraseScene(PCLVisualizer& viz, const vector<string> ids) {
  for (auto& id : ids) {
    viz.removeCorrespondences(id);
  }
}

string CESStaticScene::GetLineID(const int &id) {
  return "ces_scene_line_" + to_string(id);
}
