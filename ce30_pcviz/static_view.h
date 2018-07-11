#ifndef STATIC_VIEW_H
#define STATIC_VIEW_H

#include "scene.h"
#include "export.h"
#include <pcl/point_types.h>

namespace ce30_pcviz {
class API StaticView : public Scene
{
public:
  StaticView(std::shared_ptr<pcl::visualization::PCLVisualizer> viz);
  StaticView(
      std::shared_ptr<pcl::visualization::PCLVisualizer> viz,
      const pcl::PointXYZ& at, const pcl::PointXYZ& toward);
  StaticView(
      std::shared_ptr<pcl::visualization::PCLVisualizer> viz,
      const float& at_x, const float& at_y, const float& at_z,
      const float& toward_x, const float& toward_y, const float& toward_z);
  void Change(
      const float& at_x, const float& at_y, const float& at_z,
      const float& toward_x, const float& toward_y, const float& toward_z);
  void Change(const pcl::PointXYZ& at, const pcl::PointXYZ& toward);
  bool Change();
private:
  pcl::PointXYZ at_;
  pcl::PointXYZ toward_;
};
}

#endif // STATIC_VIEW_H
