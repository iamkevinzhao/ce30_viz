#ifndef CLOUD_CLUSTER_SCENE_H
#define CLOUD_CLUSTER_SCENE_H

#include "cloud_scene.h"
#include "export.h"

namespace ce30_pcviz {
class API CloudClusterScene : public CloudScene
{
public:
  CloudClusterScene(
      std::shared_ptr<pcl::visualization::PCLVisualizer> visualizer);
  virtual ~CloudClusterScene();
  virtual void Update();
  void DrawClusterFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
protected:
  void AddCubicFrame(
      const float x_min, const float x_max,
      const float y_min, const float y_max,
      const float z_min, const float z_max);
  void ClearAllCubicFrames();
  void Erase() override;
private:
  std::string EdgeID();
  std::vector<std::string> cubic_frame_edge_ids_;
};
} // namespace ce30_pcviz

#endif // CLOUD_CLUSTER_SCENE_H
