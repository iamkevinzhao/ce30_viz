#include "grid_scene.h"

using namespace std;
using namespace pcl::visualization;

namespace ce30_pcviz {
GridScene::GridScene(shared_ptr<PCLVisualizer> viz) : StaticScene(viz)
{
}

void GridScene::Show() {
  cout << "GridScene" << endl;
}
} // namespace ce30_pcviz
