#include "scene.h"
#include <pcl/visualization/pcl_visualizer.h>

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
  OnVisualizerLoaded(visualizer_);
  OnVisualizerLoaded();
}

void Scene::RegisterComponent(const string &id) {
  component_ids_.push_back(id);
}

void Scene::OnVisualizerLoaded(std::shared_ptr<PCLVisualizer> viz) {
}

void Scene::OnVisualizerLoaded() {}

void Scene::Erase() {
  for (auto& id : component_ids_) {
    visualizer_->removeCorrespondences(id);
  }
  for (auto& child : children_) {
    child->Erase();
  }
}
}
