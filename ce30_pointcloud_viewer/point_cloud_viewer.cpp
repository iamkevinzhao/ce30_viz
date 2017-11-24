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
}

PointCloudViewer::~PointCloudViewer() {
}

bool PointCloudViewer::Init() {
  startTimer(0);
  socket_.reset(new UDPSocket);
  if (!Connect(*socket_)) {
    cerr << "Unable to Connect Device!" << endl;
    exit_code_= ExitCode::device_connection_failure;
    return false;
  }
  if (!StartRunning(*socket_)) {
    cerr << "Unable to Start CE30" << endl;
    exit_code_= ExitCode::start_ce30_failure;
    return false;
  }
  string device_version;
  if (!GetVersion(device_version, *socket_)) {
    cerr << "Unable to Retrieve CE30 Device Version" << endl;
    exit_code_= ExitCode::retrieve_ce30_version_failure;
    return false;
  }
  return true;
}

void PointCloudViewer::timerEvent(QTimerEvent *event) {
  if (exit_code_ != ExitCode::no_exit) {
    QCoreApplication::exit((int)exit_code_);
  }
  Packet packet;
  socket_->GetPacket(packet);
}
