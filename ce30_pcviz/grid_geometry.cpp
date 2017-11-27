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
                           const float& x, const float& y, const float& size)
  : width_(width), height_(height), x_(x), y_(y), size_(size) {
  Calculate();
}
void GridGeometry::Calculate() {
  PointXYZ end_left(x_, y_, 0.0f);
  PointXYZ end_right(x_, y_ - width_ * size_, 0.0f);
  PointXYZ end_top = end_left;
  PointXYZ end_bottom(x_ - height_ * size_, y_, 0.0f);

  for (int hgt = 0; hgt <= height_; ++hgt) {
    EndToEnd e2e{end_left, end_right};
    e2e.first.x = end_left.x - size_ * hgt;
    e2e.second.x = end_right.x - size_ * hgt;
    verticals_.push_back(e2e);
  }

  for (int wid = 0; wid <= width_; ++wid) {
    EndToEnd e2e{end_top, end_bottom};
    e2e.first.y = end_top.y - size_ * wid;
    e2e.second.y = end_bottom.y - size_ * wid;
    horizontals_.push_back(e2e);
  }
}

vector<EndToEnd> GridGeometry::Horizontals() {
  return horizontals_;
}

vector<EndToEnd> GridGeometry::Verticals() {
  return verticals_;
}
}
