#include "grid_geometry.h"
#include <iostream>

using namespace pcl;
using namespace std;

//        x
//        ^
//        |
//    y<---

//    size
//    ----
//    |[]|
//    ----

//    <--- left   right --->

//  (x,y)  width
//    *------------------ h    ^
//    |[]|[]|[]|[]|[]|[]| e    |
//    ------------------- i   top
//    |[]|[]|[]|[]|[]|[]| g
//    ------------------- h  bottom
//    |[]|[]|[]|[]|[]|[]| t    |
//    -------------------      V

namespace ce30_pcviz {
GridGeometry::GridGeometry(const int& width, const int& height,
                           const float& x, const float& y,
                           const float& size)
  : GridGeometry(width, height, x, y, 0.0f, size) {
}

GridGeometry::GridGeometry(const int& width, const int& height,
                           const float& x, const float& y, const float& z,
                           const float& size)
  : width_(width), height_(height), x_(x), y_(y), z_(z), size_(size) {
  Calculate();
}

void GridGeometry::Calculate() {
  PointXYZ end_left(x_, y_, z_);
  PointXYZ end_right(x_, y_ - width_ * size_, z_);
  PointXYZ end_top = end_left;
  PointXYZ end_bottom(x_ - height_ * size_, y_, z_);

  for (int hgt = 0; hgt <= height_; ++hgt) {
    EndToEnd e2e{end_left, end_right};
    e2e.first.x = end_left.x - size_ * hgt;
    e2e.second.x = end_right.x - size_ * hgt;
    verticals_.push_back(e2e);
    if (hgt == 0) {
      top_left_ = e2e.first;
      top_right_ = e2e.second;
    } else if (hgt == height_) {
      bottom_right_ = e2e.first;
      bottom_left_ = e2e.second;
    }

    left_border_.push_back(e2e.first);
    right_border_.push_back(e2e.second);
  }

  for (int wid = 0; wid <= width_; ++wid) {
    EndToEnd e2e{end_top, end_bottom};
    e2e.first.y = end_top.y - size_ * wid;
    e2e.second.y = end_bottom.y - size_ * wid;
    horizontals_.push_back(e2e);
    top_border_.push_back(e2e.first);
    bottom_border_.push_back(e2e.second);
  }
}

vector<EndToEnd> GridGeometry::Horizontals() {
  return horizontals_;
}

vector<EndToEnd> GridGeometry::Verticals() {
  return verticals_;
}

pcl::PointXYZ GridGeometry::TopLeftCorner() {
  return top_left_;
}

pcl::PointXYZ GridGeometry::BottomLeftCorner() {
  return bottom_left_;
}

pcl::PointXYZ GridGeometry::BottomRightCorner() {
  return bottom_right_;
}

pcl::PointXYZ GridGeometry::TopRightCorner() {
  return top_right_;
}

std::vector<pcl::PointXYZ> GridGeometry::LeftBorder() {
  return left_border_;
}

std::vector<pcl::PointXYZ> GridGeometry::RightBorder() {
  return right_border_;
}

std::vector<pcl::PointXYZ> GridGeometry::TopBorder() {
  return top_border_;
}

std::vector<pcl::PointXYZ> GridGeometry::BottomBorder() {
  return bottom_border_;
}
}
