#include "ces_static_scene.h"
#include <iostream>

using namespace std;

CESStaticScene::CESStaticScene()
  : Scene(nullptr), showing_(false)
{

}

void CESStaticScene::SetShow(const bool &show) {
  showing_ = show;
}

bool CESStaticScene::Showing() {
  return showing_;
}

void CESStaticScene::Update() {
  static bool last_showing = showing_;
  if (last_showing != showing_) {
    if (!showing_) {
      Viz().addText3D("PPP", pcl::PointXYZ(1.0f, 1.0f, 1.0f), 1.0, 1.0, 1.0, 1.0, "PPP");
    } else {
      Viz().removeText3D("PPP");
    }
  }
  last_showing = showing_;
}
