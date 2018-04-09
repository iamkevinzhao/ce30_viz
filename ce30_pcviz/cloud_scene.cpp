#include "cloud_scene.h"

namespace ce30_pcviz {
CloudScene::CloudScene(std::shared_ptr<pcl::visualization::PCLVisualizer> viz) :
    Scene(viz)
{

}

CloudScene::~CloudScene() {}

void CloudScene::SetCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  cloud_ = cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudScene::GetCloud() {
  return cloud_;
}
}
