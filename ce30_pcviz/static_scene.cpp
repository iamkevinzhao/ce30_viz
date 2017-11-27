#include "static_scene.h"

using namespace std;
using namespace pcl::visualization;

namespace ce30_pcviz {
StaticScene::StaticScene(shared_ptr<PCLVisualizer> viz)
  : Scene(viz), shown_(false) {}
StaticScene::~StaticScene() {}
void StaticScene::Update() {
  if (!shown_) {
    Show();
    shown_ = true;
  }
  Scene::Update();
}
void StaticScene::Show() {}
}
