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
  virtual void Update() = 0;
protected:
  inline pcl::visualization::PCLVisualizer& Viz() {
    return *visualizer_;
  }
private:
  std::shared_ptr<pcl::visualization::PCLVisualizer> visualizer_;
};
} // namespace ce30_pcviz

#endif // SCENE_H
