#pragma once
#ifndef CE30_VIZ_CE30_PCVIZ_H
#define CE30_VIZ_CE30_PCVIZ_H

#include <ce30_pcviz/config.h>
#include "export.h"
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "scene.h"
// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include "operation_handler.h"
#include "cloud_scene.h"

namespace pcl {
  namespace visualization {
    class PCLVisualizer;
  } // namespace visualization
} // namespace pcl

namespace ce30_pcviz {
using PCLPoint = pcl::PointXYZRGB;
class API Point {
public:
  Point();
  Point(const float& x, const float& y, const float& z);
  inline const PCLPoint& pcl_point() const {
    return point_;
  }
  static void SetXRange(const float& min, const float& max);
private:
  static void RainbowColorize(
      const float& x, const float& min, const float& max,
      unsigned char& r, unsigned char& g, unsigned char& b);
  static float x_max_;
  static float x_min_;

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
  inline const void Reserve(int num) {
    cloud_.points.reserve(num);
  }
private:
  pcl::PointCloud<PCLPoint> cloud_;
};

class API PointCloudViz {
public:
  PointCloudViz();
  static std::string Version();
  void SetRefreshInterval(const int& millisecs);
  void Show();
  bool Closed();
  void UpdatePointCloud(const PointCloud& point_cloud);
  void AddCtrlShortcut(const CtrlShortcut& shortcut);
  std::vector<CtrlShortcut> GetAllCtrlShortcuts();
  std::vector<std::pair<std::string, std::string>> CtrlShortcutMap();
  void PrintShortcuts();
  static bool SavePCD(const std::string& file, const PointCloud& pointcloud);
  void AddScene(std::shared_ptr<Scene> scene);
  void UpdateWorldScene(std::shared_ptr<Scene> scene);
  void ClusterModeOn(const bool& on);
  bool IsClusterModeOn();
  std::shared_ptr<pcl::visualization::PCLVisualizer> GetPCLViz();
  void SpinOnce();
  void SetPointSize(const int& size);
  void RegisterPointPickedCallback(
      std::function<void(float x, float y, float z)>);
private:
  void OnFirstPointCloud(
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr,
      pcl::visualization::PointCloudColorHandlerRGBField<PCLPoint>& rgb);
  std::shared_ptr<Scene> world_scene_;
  std::shared_ptr<CloudScene> cloud_cluster_scene_;
  std::shared_ptr<pcl::visualization::PCLVisualizer> viz_;
  std::unique_ptr<OperationHandler> operation_;
  bool first_cloud_;
  int refresh_interval_; // in millisecs
  int cloud_point_size_;
  const std::string kVizWindowTitle;
};
}

#endif
