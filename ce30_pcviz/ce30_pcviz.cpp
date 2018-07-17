
#include "ce30_pcviz.h"
#include <iostream>
#include <string>
#include "world_scene.h"
#include "operation_handler.h"
#include <pcl/io/pcd_io.h>
#include "cloud_cluster_scene.h"
#include <pcl/visualization/pcl_visualizer.h>
#include "helper_utils.h"

using namespace std;
using namespace pcl::visualization;

namespace ce30_pcviz {
float Point::x_max_ = 30.0f;
float Point::x_min_ = 0.0f;

Point::Point() : Point(0.0f, 0.0f, 0.0f) {}
Point::Point(const float& x, const float& y, const float& z) {
  point_.x = x;
  point_.y = y;
  point_.z = z;
  RainbowColorize(point_.x, x_min_, x_max_, point_.r, point_.g, point_.b);
}

void Point::RainbowColorize(
    const float& x, const float& min, const float& max,
    unsigned char& r, unsigned char& g, unsigned char& b) {
  float lut_scale = 255.0f / (max - min);
  int value = round((x - min) * lut_scale);

  r = value > 128 ? (value - 128) * 2 : 0;
  g = value < 128 ? 2 * value : 255 - ( (value - 128) * 2);
  b = value < 128 ? 255 - (2 * value) : 0;
}

void Point::SetXRange(const float &min, const float &max) {
  x_min_ = min;
  x_max_ = max;
}

PointCloud::PointCloud() {}

PointCloudViz::PointCloudViz() :
    first_cloud_(true),
    refresh_interval_(0),
    cloud_point_size_(3),
    kVizWindowTitle("CE30 Point Cloud Viz") {
  viz_.reset(new PCLVisualizer(kVizWindowTitle));
  world_scene_.reset(new WorldScene(viz_));
//  cloud_cluster_scene_.reset(new CloudClusterScene(viz_));
  operation_.reset(new OperationHandler(viz_));
  operation_->RegisterPointPickedCallback(
      [this](float x, float y, float z){
    viz_->setWindowName("Picked Point: " + ToCoordinateString(x, y, z));
  });
  operation_->RegisterPointPickingModeOffCallback(
      [this](){
    viz_->setWindowName(kVizWindowTitle);
  });
}

string PointCloudViz::Version() {
  return CE30_VIZ_VERSION_STRING;
}

void PointCloudViz::SetRefreshInterval(const int &millisecs) {
  refresh_interval_ = millisecs;
}

bool PointCloudViz::Closed() {
  return viz_->wasStopped();
}

void PointCloudViz::UpdatePointCloud(const PointCloud &point_cloud) {
  world_scene_->Update();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (
      new pcl::PointCloud<pcl::PointXYZRGB>(point_cloud.pcl_pointcloud()));

  PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
  if (cloud_cluster_scene_) {
    cloud_cluster_scene_->SetCloud(point_cloud_ptr);
    cloud_cluster_scene_->Update();
  }

  if (first_cloud_) {
    OnFirstPointCloud(point_cloud_ptr, rgb);
    first_cloud_ = false;
  }

  viz_->updatePointCloud<pcl::PointXYZRGB>(point_cloud_ptr, rgb);

  SpinOnce();
}

void PointCloudViz::SpinOnce() {
  if (!viz_->wasStopped()) {
    viz_->spinOnce(refresh_interval_);
  }
}

void PointCloudViz::SetPointSize(const int &size) {
  cloud_point_size_ = size;
}

void PointCloudViz::RegisterPointPickedCallback(
    std::function<void (float, float, float)> callback) {
  operation_->RegisterPointPickedCallback(callback);
}

void PointCloudViz::AddCtrlShortcut(const CtrlShortcut &shortcut) {
  return operation_->AddShortcut(shortcut);
}

std::vector<CtrlShortcut> PointCloudViz::GetAllCtrlShortcuts() {
  return operation_->GetAllCtrlShortcuts();
}

vector<pair<string, string>> PointCloudViz::CtrlShortcutMap() {
  return operation_->CtrlShortcutMap();
}

void PointCloudViz::PrintShortcuts() {
  return operation_->PrintShortcuts();
}

bool PointCloudViz::SavePCD(const string &file, const PointCloud &pointcloud) {
  return pcl::io::savePCDFileASCII(file, pointcloud.pcl_pointcloud());
}

void PointCloudViz::AddScene(std::shared_ptr<Scene> scene) {
  if (!scene->VisualizerLoaded()) {
    scene->LoadVisualizer(viz_);
  }
  world_scene_->AddChild(scene);
}

void PointCloudViz::UpdateWorldScene(std::shared_ptr<Scene> scene) {
  if (!scene->VisualizerLoaded()) {
    scene->LoadVisualizer(viz_);
  }
  world_scene_->Erase();
  world_scene_ = scene;
}

void PointCloudViz::OnFirstPointCloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr,
    PointCloudColorHandlerRGBField<PCLPoint>& rgb) {
  viz_->addPointCloud<pcl::PointXYZRGB>(point_cloud_ptr, rgb);
  viz_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloud_point_size_);
  viz_->setShowFPS(false);
  operation_->UseAerialView();
}

void PointCloudViz::ClusterModeOn(const bool &on) {
  if (cloud_cluster_scene_) {
    if (!on) {
      cloud_cluster_scene_->Erase();
      cloud_cluster_scene_.reset();
    }
  } else {
    if (on) {
      if (viz_) {
        cloud_cluster_scene_.reset(new CloudClusterScene(viz_));
      }
    }
  }
}

bool PointCloudViz::IsClusterModeOn() {
  return cloud_cluster_scene_ != nullptr;
}

std::shared_ptr<pcl::visualization::PCLVisualizer> PointCloudViz::GetPCLViz() {
  return viz_;
}
}
