#include "cloud_cluster_scene.h"
#include <iostream>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace ce30_pcviz {
CloudClusterScene::CloudClusterScene(
    std::shared_ptr<pcl::visualization::PCLVisualizer> visualizer)
  : CloudScene(visualizer)
{

}

CloudClusterScene::~CloudClusterScene() {}

void CloudClusterScene::Update() {
  auto cloud = GetCloud();
  if (cloud) {
    DrawClusterFrame(cloud);
  }
  CloudScene::Update();
}

void CloudClusterScene::DrawClusterFrame(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->reserve(cloud_rgb->size());
  for (auto& rgb : *cloud_rgb) {
    cloud->push_back({rgb.x, rgb.y, rgb.z});
  }

  auto cloud_filtered = cloud;
  if (cloud->empty()) {
    return;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.4);
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (100000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  if (!cluster_indices.empty()) {
    ClearAllCubicFrames();
  }

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
//    static float min = std::numeric_limits<float>::min();
//    static float max = std::numeric_limits<float>::max();
    static float min = -1000;
    static float max = 1000;

    float x_min = max, x_max = min;
    float y_min = max, y_max = min;
    float z_min = max, z_max = min;

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
      auto point = cloud_filtered->points[*pit];
      if (x_min > point.x) x_min = point.x;
      if (y_min > point.y) y_min = point.y;
      if (z_min > point.z) z_min = point.z;
      if (x_max < point.x) x_max = point.x;
      if (y_max < point.y) y_max = point.y;
      if (z_max < point.z) z_max = point.z;
    }

    AddCubicFrame(x_min, x_max, y_min, y_max, z_min, z_max);
  }
}

void CloudClusterScene::ClearAllCubicFrames() {
  for (auto& id : cubic_frame_edge_ids_) {
    Viz().removeCorrespondences(id);
  }
  cubic_frame_edge_ids_.clear();
}

void CloudClusterScene::Erase() {
  ClearAllCubicFrames();
}

std::string CloudClusterScene::EdgeID() {
  auto id =
      "cubic_frame_line_" +
      std::to_string(cubic_frame_edge_ids_.size());
  cubic_frame_edge_ids_.push_back(id);
  return id;
}

void CloudClusterScene::AddCubicFrame(
    const float x_min, const float x_max,
    const float y_min, const float y_max,
    const float z_min, const float z_max) {
  static float r = 0, g = 1, b = 0;
  Viz().addLine(
      pcl::PointXYZ{x_max, y_min, z_min},
      pcl::PointXYZ{x_max, y_max, z_min},
      r, g, b, EdgeID());
  Viz().addLine(
      pcl::PointXYZ{x_max, y_min, z_min},
      pcl::PointXYZ{x_max, y_min, z_max},
      r, g, b, EdgeID());
  Viz().addLine(
      pcl::PointXYZ{x_max, y_min, z_max},
      pcl::PointXYZ{x_max, y_max, z_max},
      r, g, b, EdgeID());
  Viz().addLine(
      pcl::PointXYZ{x_max, y_min, z_max},
      pcl::PointXYZ{x_min, y_min, z_max},
      r, g, b, EdgeID());
  Viz().addLine(
      pcl::PointXYZ{x_max, y_max, z_min},
      pcl::PointXYZ{x_max, y_max, z_max},
      r, g, b, EdgeID());
  Viz().addLine(
      pcl::PointXYZ{x_min, y_min, z_max},
      pcl::PointXYZ{x_min, y_min, z_min},
      r, g, b, EdgeID());
  Viz().addLine(
      pcl::PointXYZ{x_max, y_min, z_min},
      pcl::PointXYZ{x_min, y_min, z_min},
      r, g, b, EdgeID());
  Viz().addLine(
      pcl::PointXYZ{x_min, y_max, z_max},
      pcl::PointXYZ{x_min, y_min, z_max},
      r, g, b, EdgeID());
  Viz().addLine(
      pcl::PointXYZ{x_min, y_max, z_max},
      pcl::PointXYZ{x_min, y_max, z_min},
      r, g, b, EdgeID());
  Viz().addLine(
      pcl::PointXYZ{x_min, y_max, z_min},
      pcl::PointXYZ{x_max, y_max, z_min},
      r, g, b, EdgeID());
  Viz().addLine(
      pcl::PointXYZ{x_min, y_max, z_max},
      pcl::PointXYZ{x_max, y_max, z_max},
      r, g, b, EdgeID());
  Viz().addLine(
      pcl::PointXYZ{x_min, y_min, z_min},
      pcl::PointXYZ{x_min, y_max, z_min},
      r, g, b, EdgeID());
}
} // namespace ce30_pcviz
