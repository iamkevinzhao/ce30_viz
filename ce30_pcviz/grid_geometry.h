#ifndef GRID_GEOMETRY_H
#define GRID_GEOMETRY_H

#include <vector>
#include <pcl/point_types.h>

namespace ce30_pcviz {
using EndToEnd = std::pair<pcl::PointXYZ, pcl::PointXYZ>;

class GridGeometry
{
public:
  GridGeometry(
      const int& width, const int& height,
      const float& x, const float& y, const float& size);
  GridGeometry(
      const int& width, const int& height,
      const float& x, const float& y, const float& z,
      const float& size);
  std::vector<EndToEnd> Horizontals();
  std::vector<EndToEnd> Verticals();
  pcl::PointXYZ TopLeftCorner();
  pcl::PointXYZ BottomLeftCorner();
  pcl::PointXYZ BottomRightCorner();
  pcl::PointXYZ TopRightCorner();
  std::vector<pcl::PointXYZ> LeftBorder();
  std::vector<pcl::PointXYZ> RightBorder();
  std::vector<pcl::PointXYZ> TopBorder();
  std::vector<pcl::PointXYZ> BottomBorder();
private:
  void Calculate();
  int width_;
  int height_;
  float x_;
  float y_;
  float z_;
  float size_;

  std::vector<EndToEnd> horizontals_;
  std::vector<EndToEnd> verticals_;

  pcl::PointXYZ top_left_;
  pcl::PointXYZ bottom_left_;
  pcl::PointXYZ bottom_right_;
  pcl::PointXYZ top_right_;

  std::vector<pcl::PointXYZ> left_border_;
  std::vector<pcl::PointXYZ> right_border_;
  std::vector<pcl::PointXYZ> top_border_;
  std::vector<pcl::PointXYZ> bottom_border_;
};
}

#endif // GRID_GEOMETRY_H
