#pragma once
#ifndef CE30_VIZ_CE30_PCVIZ_H
#define CE30_VIZ_CE30_PCVIZ_H

#include <ce30_pcviz/config.h>
#include "export.h"
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "scene.h"
#include <pcl/visualization/pcl_visualizer.h>

namespace ce30_pcviz {
using PCLPoint = pcl::PointXYZRGB;
class API Point {
public:
  Point();
  Point(const float& x, const float& y, const float& z);
  inline const PCLPoint& pcl_point() const {
    return point_;
  }
private:
  PCLPoint point_;
};

class API PointCloud {
public:
  PointCloud();
  inline void push_back(const Point& point) {
    return cloud_.push_back(point.pcl_point());
  }
  inline const pcl::PointCloud<PCLPoint>& pcl_pointcloud() const {
    return cloud_;
  }
private:
  pcl::PointCloud<PCLPoint> cloud_;
};

class API PointCloudViz {
public:
  PointCloudViz();
  static std::string Version();
  void Show();
  bool Closed();
  void UpdatePointCloud(const PointCloud& point_cloud);
private:
  std::unique_ptr<Scene> world_scene_;
  std::shared_ptr<pcl::visualization::PCLVisualizer> viz_;
};
}

#endif
