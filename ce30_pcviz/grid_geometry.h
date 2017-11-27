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
  std::vector<EndToEnd> Horizontals();
  std::vector<EndToEnd> Verticals();
private:
  void Calculate();
  int width_;
  int height_;
  float x_;
  float y_;
  float size_;

  std::vector<EndToEnd> horizontals_;
  std::vector<EndToEnd> verticals_;
};
}

#endif // GRID_GEOMETRY_H
