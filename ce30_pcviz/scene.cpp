#include "scene.h"

using namespace std;
using namespace pcl::visualization;

namespace ce30_pcviz {
Scene::Scene(shared_ptr<PCLVisualizer> visualizer) : visualizer_(visualizer) {}
Scene::~Scene() {}
}
