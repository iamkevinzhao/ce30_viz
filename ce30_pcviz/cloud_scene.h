#ifndef CLOUD_SCENE_H
#define CLOUD_SCENE_H

#include "scene.h"
#include "export.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace ce30_pcviz {
class API CloudScene : public Scene
{
public:
  CloudScene(std::shared_ptr<pcl::visualization::PCLVisualizer> viz);
  virtual ~CloudScene();
  void SetCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
protected:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetCloud();
private:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
};
} // namespace ce30_pcviz

#endif // CLOUD_SCENE_H
