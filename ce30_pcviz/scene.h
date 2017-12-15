#ifndef SCENE_H
#define SCENE_H

#include "export.h"
#include <memory>
#include <pcl/visualization/pcl_visualizer.h>

namespace ce30_pcviz {
class API Scene
{
public:
  Scene(std::shared_ptr<pcl::visualization::PCLVisualizer> visualizer);
  virtual ~Scene();
  virtual void Update();
  bool VisualizerLoaded();
  void LoadVisualizer(
      std::shared_ptr<pcl::visualization::PCLVisualizer> visualizer);
  virtual void AddChild(std::shared_ptr<Scene> child);
protected:
  inline pcl::visualization::PCLVisualizer& Viz() {
    return *visualizer_;
  }
  inline std::shared_ptr<pcl::visualization::PCLVisualizer> VizPtr() {
    return visualizer_;
  }
private:
  std::shared_ptr<pcl::visualization::PCLVisualizer> visualizer_;
  std::vector<std::shared_ptr<Scene>> children_;
};
} // namespace ce30_pcviz

#endif // SCENE_H
