
#include "ce30_pcviz.h"
#include <iostream>
#include <string>
#include "world_scene.h"

using namespace std;
using namespace pcl::visualization;

#include <random>
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> x_dis(5.0, 10.0);
std::uniform_real_distribution<> y_dis(-10, 10.0);
std::uniform_real_distribution<> z_dis(-3.0, 3.0);

namespace ce30_pcviz {
Point::Point() : Point(0.0f, 0.0f, 0.0f) {}
Point::Point(const float& x, const float& y, const float& z) {
  point_.x = x;
  point_.y = y;
  point_.z = z;

  point_.x = x_dis(gen);
  point_.y = y_dis(gen);
  point_.z = z_dis(gen);

  point_.r = 100;
}

PointCloud::PointCloud() {}

PointCloudViz::PointCloudViz() {
  viz_.reset(new PCLVisualizer("CE30 Point Cloud Viz"));
  world_scene_.reset(new WorldScene(viz_));
}

string PointCloudViz::Version() {
  return CE30_VIZ_VERSION_STRING;
}

void PointCloudViz::Show() {

}

bool PointCloudViz::Closed() {
  return viz_->wasStopped();
}

void PointCloudViz::UpdatePointCloud(const PointCloud &point_cloud) {
  world_scene_->Update();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>(point_cloud.pcl_pointcloud()));

  for (int i = 0; i < 100; ++i) {
    point_cloud_ptr->push_back(Point().pcl_point());
  }

  PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
  static bool called = false;
  if (!called) {
    viz_->addPointCloud<pcl::PointXYZRGB>(point_cloud_ptr, rgb, "sample cloud");
    viz_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "sample cloud");
    called = true;
  }

  viz_->updatePointCloud<pcl::PointXYZRGB>(point_cloud_ptr, rgb, "sample cloud");

  if (!viz_->wasStopped()) {
    viz_->spinOnce(100);
  }
}
}
