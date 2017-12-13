#include "scene.h"

using namespace std;
using namespace pcl::visualization;

namespace ce30_pcviz {
Scene::Scene(shared_ptr<PCLVisualizer> visualizer) : visualizer_(visualizer) {}
Scene::~Scene() {}
void Scene::AddChild(std::shared_ptr<Scene> child) {
  children_.push_back(child);
}
void Scene::Update() {
  for (auto& child : children_) {
    child->Update();
  }
}

bool Scene::VisualizerLoaded() {
  return visualizer_ != nullptr;
}

void Scene::LoadVisualizer(std::shared_ptr<PCLVisualizer> visualizer) {
  visualizer_ = visualizer;
}
}
