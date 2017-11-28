
#include "ce30_pcviz.h"
#include <iostream>
#include <string>
#include "world_scene.h"
#include "operation_handler.h"

using namespace std;
using namespace pcl::visualization;

#include <random>
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> x_dis(5.0, 10.0);
std::uniform_real_distribution<> y_dis(-10, 10.0);
std::uniform_real_distribution<> z_dis(-3.0, 3.0);

namespace ce30_pcviz {
float Point::x_max_ = 10.0f;
float Point::x_min_ = 5.0f;

Point::Point() : Point(0.0f, 0.0f, 0.0f) {}
Point::Point(const float& x, const float& y, const float& z) {
  point_.x = x;
  point_.y = y;
  point_.z = z;

//  point_.x = x_dis(gen);
//  point_.y = y_dis(gen);
//  point_.z = z_dis(gen);

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

PointCloudViz::PointCloudViz() : first_cloud_(true) {
  viz_.reset(new PCLVisualizer("CE30 Point Cloud Viz"));
  world_scene_.reset(new WorldScene(viz_));
  operation_.reset(new OperationHandler(viz_));
}

string PointCloudViz::Version() {
  return CE30_VIZ_VERSION_STRING;
}

bool PointCloudViz::Closed() {
  return viz_->wasStopped();
}

void PointCloudViz::UpdatePointCloud(const PointCloud &point_cloud) {
  world_scene_->Update();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (
      new pcl::PointCloud<pcl::PointXYZRGB>(point_cloud.pcl_pointcloud()));
//  for (int i = 0; i < 100; ++i) {
//    point_cloud_ptr->push_back(Point().pcl_point());
//  }

  PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
  if (first_cloud_) {
    OnFirstPointCloud(point_cloud_ptr, rgb);
    first_cloud_ = false;
  }

  viz_->updatePointCloud<pcl::PointXYZRGB>(point_cloud_ptr, rgb);

  if (!viz_->wasStopped()) {
    viz_->spinOnce(10);
  }
}

void PointCloudViz::OnFirstPointCloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr,
    PointCloudColorHandlerRGBField<PCLPoint>& rgb) {
  viz_->addPointCloud<pcl::PointXYZRGB>(point_cloud_ptr, rgb);
  viz_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7);
  operation_->UseAerialView();
}
}
