#include "static_view.h"
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl::visualization;

namespace ce30_pcviz {
StaticView::StaticView(shared_ptr<PCLVisualizer> viz)
  : Scene(viz), at_(0.0f, 0.0f, 0.0f), toward_(0.0f, 0.0f, 0.0f) {}

StaticView::StaticView(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viz,
    const pcl::PointXYZ& at, const pcl::PointXYZ& toward)
  : StaticView(viz) {
  at_ = at;
  toward_ = toward;
}

StaticView::StaticView(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viz,
    const float& at_x, const float& at_y, const float& at_z,
    const float& toward_x, const float& toward_y, const float& toward_z)
  : StaticView(
        viz,
        pcl::PointXYZ(at_x, at_y, at_z),
        pcl::PointXYZ(toward_x, toward_y, toward_z)) {}

void StaticView::Change(
    const float& at_x, const float& at_y, const float& at_z,
    const float& toward_x, const float& toward_y, const float& toward_z) {
  Change(
      pcl::PointXYZ(at_x, at_y, at_z),
      pcl::PointXYZ(toward_x, toward_y, toward_z));
}

void StaticView::Change(const pcl::PointXYZ &at, const pcl::PointXYZ &toward) {
  Viz().setCameraPosition(at.x, at.y, at.z, toward.x, toward.y, toward.z, 0.0f, 0.0f, 1.0f);
}

bool StaticView::Change() {
  if (at_.x == 0.0f &&
      at_.y == 0.0f &&
      at_.z == 0.0f &&
      toward_.x == 0.0f &&
      toward_.y == 0.0f &&
      toward_.z == 0.0f) {
    return false;
  }
  Change(at_, toward_);
  return true;
}
}

