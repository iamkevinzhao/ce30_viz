#include "point_cloud_viewer.h"
#include <iostream>
#include <ce30_pcviz/ce30_pcviz.h>
#include <ce30_driver/utils.h>
#include <QCoreApplication>
#include <QThread>

using namespace std;
using namespace ce30_pcviz;
using namespace ce30_driver;

PointCloudViewer::PointCloudViewer()
{
  startTimer(0);
  ce30_pcviz::Point::SetXRange(Channel::DistanceMin(), Channel::DistanceMax());
}

PointCloudViewer::~PointCloudViewer() {
  StopRunning(*socket_);
}

ExitCode PointCloudViewer::ConnectOrExit(UDPSocket& socket) {
  if (!Connect(socket)) {
    cerr << "Unable to Connect Device!" << endl;
    return ExitCode::device_connection_failure;
  }
  if (!StartRunning(socket)) {
    cerr << "Unable to Start CE30" << endl;
    return ExitCode::start_ce30_failure;
  }
  string device_version;
  if (!GetVersion(device_version, socket)) {
    cerr << "Unable to Retrieve CE30 Device Version" << endl;
    return ExitCode::retrieve_ce30_version_failure;
  }
  cout << "CE30 Version: " << device_version << endl;
  return ExitCode::no_exit;
}

void PointCloudViewer::timerEvent(QTimerEvent *event) {
  if (!socket_) {
    socket_.reset(new UDPSocket);
    auto ec = ConnectOrExit(*socket_);
    if (ec != ExitCode::no_exit) {
      QThread::sleep(2);
      QCoreApplication::exit((int)ec);
      return;
    }
  }
  if (!pcviz_) {
    pcviz_.reset(new PointCloudViz);
  }
  if (pcviz_->Closed()) {
     QCoreApplication::exit((int)ExitCode::normal_exit);
    return;
  }

//  pcviz_->UpdatePointCloud(PointCloud());
//  return;

  Packet packet;
  if (GetPacket(packet, *socket_)) {
    auto parsed = packet.Parse();
    if (parsed) {
      scan_.AddColumnsFromPacket(*parsed);
      if (scan_.Ready()) {
        UpdatePointCloudDisplay(scan_, *pcviz_);
        scan_.Reset();
      }
    }
  }
}

#ifndef FAKE_POINTCLOUD
void PointCloudViewer::UpdatePointCloudDisplay(
    const Scan &scan, PointCloudViz &viz) {
  PointCloud cloud;
  for (int x = 0; x < scan.Width(); ++x) {
    for (int y = 0; y < scan.Height(); ++y) {
      ce30_driver::Point p = scan.at(x, y).point();
      cloud.push_back(ce30_pcviz::Point(p.x, p.y, p.z));
    }
  }
  viz.UpdatePointCloud(cloud);
}
#else

#include <random>

std::random_device gRd;
std::mt19937 gGen(gRd());

inline static float to_rad(const float& deg) {
  return deg * M_PI / 180.0f;
}

void PointCloudViewer::UpdatePointCloudDisplay(
    const Scan &scan, PointCloudViz &viz) {
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
  viz.UpdatePointCloud(cloud);
}
#endif
