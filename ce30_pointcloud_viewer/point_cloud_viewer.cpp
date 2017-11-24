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
      QCoreApplication::exit((int)ec);
      return;
    }
  }
  if (!pcviz_) {
    pcviz_.reset(new PointCloudViz);
    pcviz_->Show();
  }
  if (pcviz_->Closed()) {
    QCoreApplication::exit((int)ExitCode::normal_exit);
  }
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

}
