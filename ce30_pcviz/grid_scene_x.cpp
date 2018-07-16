#include "grid_scene_x.h"
#include "grid_geometry.h"
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

namespace ce30_pcviz {
GridSceneX::GridSceneX(std::shared_ptr<pcl::visualization::PCLVisualizer> viz)
  : GridScene(viz)
{

}

void GridSceneX::ShowDefault() {
  int width = 20;
  int height = 200;
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

//  // Fov Boundary Lines
//  string fov_l_id = "fov_left";
//  Viz().addLine(
//        pcl::PointXYZ(0.0f, 0.0f, 0.0f),
//        pcl::PointXYZ(width * size / 2 * tan(M_PI / 3), width * size / 2, 0.0f),
//        1.0, 0.0, 0.0, fov_l_id);
//  RegisterComponent(fov_l_id);
//  string fov_r_id = "fov_right";
//  Viz().addLine(
//        pcl::PointXYZ(0.0f, 0.0f, 0.0f),
//        pcl::PointXYZ(width * size / 2 * tan(M_PI / 3), -width * size / 2, 0.0f),
//        1.0, 0.0, 0.0,
//        "fov_right");
//  RegisterComponent(fov_r_id);
}
} // namespace ce30_pcviz
