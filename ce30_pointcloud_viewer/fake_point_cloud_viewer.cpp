#include "fake_point_cloud_viewer.h"
#include <random>
#include <QCoreApplication>

std::random_device gRd;
std::mt19937 gGen(gRd());

using namespace ce30_pcviz;
using namespace std;

inline static float to_rad(const float& deg) {
  return deg * M_PI / 180.0f;
}


FakePointCloudViewer::FakePointCloudViewer()
{
  timer_id_ = startTimer(10);
  viz_.AddCtrlShortcut({"i", [](){cout << "ii" << endl;}, "ii"});
  auto key_maps = viz_.CtrlShortcutMap();
  cout << "Shortcuts: " << endl;
  for (auto key_map : key_maps) {
    cout << "  Ctrl+'" << key_map.first << "' -- " << key_map.second << endl;
  }
}

void FakePointCloudViewer::timerEvent(QTimerEvent *event) {
  if (event->timerId() == timer_id_) {
    if (viz_.Closed()) {
      QCoreApplication::exit(0);
    }
    ExecuteCycle();
  }
}

void FakePointCloudViewer::ExecuteCycle() {
  PointCloud cloud;

  float delta_y = 0.1;
  // Bulk Alpha
  {
    uniform_real_distribution<> x_dis(-0.00, 0.00);
//    uniform_real_distribution<> x_dis(-0.03, 0.03);
    uniform_real_distribution<> y_dis(-0.00, 0.00);
    uniform_real_distribution<> z_dis(-0.00, 0.00);

    float y_left = -5.0f;
    float y_right = -3.0f;
    float x_left = abs(y_left) * tan(to_rad(60));
    float x_right = x_left + 1.5f;
    float tan_alpha = (x_right - x_left) / (y_right - y_left);
    int i = 0;
    for (float y = y_left; y <= y_right; y += delta_y) {
      float x = x_left + delta_y * tan_alpha * (++i);
      static float z_span = 0.2f;
      static float z_down = -(z_span / 2);
      static float z_delta = z_span / 20;
      for (int z_index = 0; z_index < 20; ++z_index) {
        float z = z_down + z_index * z_delta;
        cloud.push_back(ce30_pcviz::Point(x + x_dis(gGen), -y + y_dis(gGen), z + z_dis(gGen)));
      }
    }
  }

  // Bulk Bravo
  {
    uniform_real_distribution<> x_dis(-0.0, 0.00);
//    uniform_real_distribution<> x_dis(-0.07, 0.07);
    uniform_real_distribution<> y_dis(-0.00, 0.00);
    uniform_real_distribution<> z_dis(-0.00, 0.00);

    float y_left = -4.0f;
    float y_right = 0.0f;
    float x_left = 29.0f;
    float x_right = 27.0f;
    float tan_bulk = (x_right - x_left) / (y_right - y_left);
    int i = 0;
    for (float y = y_left; y <= y_right; y += delta_y) {
      float x = x_left + delta_y * tan_bulk * (++i);
      static float z_span = 0.8f;
      static float z_down = -(z_span / 2);
      static float z_delta = z_span / 20;
      for (int z_index = 0; z_index < 20; ++z_index) {
        float z = z_down + z_index * z_delta;
        cloud.push_back(ce30_pcviz::Point(x + x_dis(gGen), -y + y_dis(gGen), z + z_dis(gGen)));
      }
    }
  }

  // Bulk Charlie
  {
    uniform_real_distribution<> x_dis(-0.00, 0.00);
//    uniform_real_distribution<> x_dis(-0.07, 0.07);
    uniform_real_distribution<> y_dis(-0.00, 0.00);
    uniform_real_distribution<> z_dis(-0.00, 0.00);

    float y_left = 0.0f;
    float y_right = 2.0f;
    float x_left = 27.0f;
    float x_right = 29.0f;
    float tan_bulk = (x_right - x_left) / (y_right - y_left);
    int i = 0;
    for (float y = y_left; y <= y_right; y += delta_y) {
      float x = x_left + delta_y * tan_bulk * (++i);
      static float z_span = 0.8f;
      static float z_down = -(z_span / 2);
      static float z_delta = z_span / 20;
      for (int z_index = 0; z_index < 20; ++z_index) {
        float z = z_down + z_index * z_delta;
        cloud.push_back(ce30_pcviz::Point(x + x_dis(gGen), -y + y_dis(gGen), z + z_dis(gGen)));
      }
    }
  }

  // Bulk Delta
  {
    uniform_real_distribution<> x_dis(-0.00, 0.00);
//    uniform_real_distribution<> x_dis(-0.05, 0.05);
    uniform_real_distribution<> y_dis(-0.00, 0.00);
    uniform_real_distribution<> z_dis(-0.00, 0.00);

    float y_left = 2.0f;
    float y_right = 8.0f;
    float y_bisect = 5.0f;
    float x_right = abs(y_right) * tan(to_rad(60));
    float x_left = x_right;
    float tan_alpha = (x_right - x_left) / (y_right - y_left);
    int i = 0;
    for (float y = y_left; y <= y_right; y += delta_y) {
      float x = x_left + delta_y * tan_alpha * (++i);
      static float z_span = 0.6f;
      static float z_down = -(z_span / 2);
      static float z_delta = z_span / 20;
      for (int z_index = 0; z_index < 20; ++z_index) {
        float z = z_down + z_index * z_delta;
        if (y >= y_left && y <= y_bisect) {
          if (z_index >= 4) {
            continue;
          }
        }
        cloud.push_back(ce30_pcviz::Point(x + x_dis(gGen), -y + y_dis(gGen), z + z_dis(gGen)));
      }
    }
  }
  viz_.UpdatePointCloud(cloud);
}
