#include "grid_scene.h"
#include "grid_geometry.h"
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl::visualization;

namespace ce30_pcviz {
GridScene::GridScene(shared_ptr<PCLVisualizer> viz)
  : StaticScene(viz), show_default_(true)
{
}

GridScene::~GridScene() {}

void GridScene::SetShowDefault(const bool &show_default) {
  show_default_ = show_default;
}

void GridScene::SetParams(
    const int &width, const int &height, const float &size,
    const float &x, const float &y, const float &z) {
  width_ = width;
  height_ = height;
  size_ = size;
  x_ = x;
  y_ = y;
  z_ = z;
  SetShowDefault(false);
}

void GridScene::Show() {
  if (show_default_) {
    ShowDefault();
    return;
  }
  GridGeometry grid_geo(width_, height_, x_, y_, z_, size_);
  auto horizontals = grid_geo.Horizontals();
  auto verticals = grid_geo.Verticals();
  int index = 0;
  for (auto& hor : horizontals) {
    auto id = "grid_line_h_" + to_string(index++);
    Viz().addLine(hor.first, hor.second, id);
    RegisterComponent(id);
  }
  index = 0;
  for (auto& ver : verticals) {
    auto id = "grid_line_v_" + to_string(index++);
    Viz().addLine(ver.first, ver.second, id);
    RegisterComponent(id);
  }
}

void GridScene::ShowDefault() {
  int width = 30;
  int height = 30;
  float size = 1.0f;
  float x = height * size;
  float y = width * size / 2;
  GridGeometry grid_geo(width, height, x, y, size);
  auto horizontals = grid_geo.Horizontals();
  auto verticals = grid_geo.Verticals();
  int index = 0;
  for (auto& hor : horizontals) {
    auto id = "grid_line_h_" + to_string(index++);
    Viz().addLine(hor.first, hor.second, id);
    RegisterComponent(id);
  }
  index = 0;
  for (auto& ver : verticals) {
    auto id = "grid_line_v_" + to_string(index++);
    Viz().addLine(ver.first, ver.second, id);
    RegisterComponent(id);
  }

  const float offset = 0.5;
  auto right_border = grid_geo.RightBorder();
  for (int i = 0; i <= height; i += 10) {
    auto p = right_border[i];
    auto pp = p;
    pp.y -= offset;
    stringstream ss;
    ss << fixed << setprecision(1) << p.x;
    auto id = "grid_landmark_" + to_string(i);
    Viz().addText3D(
        ss.str() + "m", pp, 0.5, 1.0, 1.0, 1.0, id);
    RegisterComponent(id);
  }

  // Fov Boundary Lines
  string fov_l_id = "fov_left";
  Viz().addLine(
        pcl::PointXYZ(0.0f, 0.0f, 0.0f),
        pcl::PointXYZ(width * size / 2 * tan(M_PI / 3), width * size / 2, 0.0f),
        1.0, 0.0, 0.0, fov_l_id);
  RegisterComponent(fov_l_id);
  string fov_r_id = "fov_right";
  Viz().addLine(
        pcl::PointXYZ(0.0f, 0.0f, 0.0f),
        pcl::PointXYZ(width * size / 2 * tan(M_PI / 3), -width * size / 2, 0.0f),
        1.0, 0.0, 0.0,
        "fov_right");
  RegisterComponent(fov_r_id);
}
} // namespace ce30_pcviz
