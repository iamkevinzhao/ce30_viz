#include "point_cloud_viewer.h"
#include <iostream>
#include <ce30_pcviz/ce30_pcviz.h>
#include <ce30_driver/utils.h>
#include <QCoreApplication>

using namespace std;
using namespace ce30_pcviz;
using namespace ce30_driver;

PointCloudViewer::PointCloudViewer()
{
  startTimer(0);
}

PointCloudViewer::~PointCloudViewer() {
//  StopRunning(*socket_);
}

ExitCode PointCloudViewer::ConnectOrExit(UDPSocket& socket) {
//  if (!Connect(socket)) {
//    cerr << "Unable to Connect Device!" << endl;
//    return ExitCode::device_connection_failure;
//  }
//  if (!StartRunning(socket)) {
//    cerr << "Unable to Start CE30" << endl;
//    return ExitCode::start_ce30_failure;
//  }
//  string device_version;
//  if (!GetVersion(device_version, socket)) {
//    cerr << "Unable to Retrieve CE30 Device Version" << endl;
//    return ExitCode::retrieve_ce30_version_failure;
//  }
//  cout << "CE30 Version: " << device_version << endl;
  return ExitCode::no_exit;
}

void PointCloudViewer::timerEvent(QTimerEvent *event) {
//  if (!socket_) {
//    socket_.reset(new UDPSocket);
//    auto ec = ConnectOrExit(*socket_);
//    if (ec != ExitCode::no_exit) {
//      QCoreApplication::exit((int)ec);
//      return;
//    }
//  }
  if (!pcviz_) {
    pcviz_.reset(new PointCloudViz);
  }
  if (pcviz_->Closed()) {
     QCoreApplication::exit((int)ExitCode::normal_exit);
    return;
  }

  pcviz_->UpdatePointCloud(PointCloud());
  return;

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
