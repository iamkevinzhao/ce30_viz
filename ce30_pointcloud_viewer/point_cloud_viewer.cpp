#include "point_cloud_viewer.h"
#include <iostream>
#include <ce30_pcviz/ce30_pcviz.h>
#include <ce30_driver/utils.h>
#include <QCoreApplication>
#include <QThread>
#include <QTime>
#include <QDir>

using namespace std;
using namespace ce30_pcviz;
using namespace ce30_driver;

PointCloudViewer::PointCloudViewer()
  : vertical_stretch_mode_(false), save_pcd_(false), use_filter_(false)
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
  string device_version;
  if (!GetVersion(device_version, socket)) {
    cerr << "Unable to Retrieve CE30 Device Version" << endl;
    return ExitCode::retrieve_ce30_version_failure;
  }
  cout << "CE30 Version: " << device_version << endl;
  if (!StartRunning(socket)) {
    cerr << "Unable to Start CE30" << endl;
    return ExitCode::start_ce30_failure;
  }
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
    OnPCVizInitialized();
  }
  if (pcviz_->Closed()) {
     QCoreApplication::exit((int)ExitCode::normal_exit);
    return;
  }

  Packet packet;
  if (GetPacket(packet, *socket_)) {
    auto parsed = packet.Parse();
    if (parsed) {
      scan_.AddColumnsFromPacket(*parsed);
      if (scan_.Ready()) {
        UpdatePointCloudDisplay(
            scan_, *pcviz_, vertical_stretch_mode_, save_pcd_);
        scan_.Reset();
      }
    }
  }
}

void PointCloudViewer::UpdatePointCloudDisplay(
    const Scan &scan,
    PointCloudViz &viz,
    const bool& vsmode,
    const bool& save_pcd) {
  PointCloud cloud;
  for (int x = 0; x < scan.Width(); ++x) {
    for (int y = 0; y < scan.Height(); ++y) {
      ce30_driver::Point p = scan.at(x, y).point();
      if (vsmode) {
        p.z = (scan.Height() - y) * 0.1f;
      }
      if (sqrt(p.x * p.x + p.y * p.y) < 0.1f) {
        continue;
      }
      cloud.push_back(ce30_pcviz::Point(p.x, p.y, p.z));
    }
  }
  viz.UpdatePointCloud(cloud);
  if (save_pcd) {
    static const QString data_dir_name = "data";
    if (!QDir(data_dir_name).exists()) {
      QDir::current().mkdir(data_dir_name);
    }
    viz.SavePCD(
        data_dir_name.toStdString() + "/" +
        QTime::currentTime().toString().replace(":", "_").toStdString() +
        ".pcd", cloud);
  }
}

void PointCloudViewer::OnPCVizInitialized() {
#ifdef CES_SPECIAL
  ces_static_scene_.reset(new CESStaticScene);
  pcviz_->AddScene(ces_static_scene_);
#endif

  pcviz_->AddCtrlShortcut(
      {"t",
       [this](){vertical_stretch_mode_ = !vertical_stretch_mode_;},
       "Switch Normal/Stretched"});
  pcviz_->AddCtrlShortcut(
      {"l",
       [this](){
         save_pcd_ = !save_pcd_;
         if (save_pcd_) {
           cout << "Recording..." << endl;
         } else {
           cout << "Recording Ended" << endl;
           static bool prompt = true;
           if (prompt) {
             cout << "  * Data Have Been Saved Under 'data' Folder" << endl;
             prompt = false;
           }
         }
       }, "Save Cloud to Disk"});

#ifdef USE_FEATURE_FILTER
  pcviz_->AddCtrlShortcut(
      {"f",
       [this](){
         StopRunning(*socket_);
         if (!use_filter_) {
           cout << "Enabling Filter" << endl;
           use_filter_ = EnableFilter(*socket_);
           if (use_filter_) {
             cout << "  * Filter On" << endl;
           } else {
             cout << "  * Failed" << endl;
           }
         } else {
           cout << "Disabling Filter" << endl;
           use_filter_ = !DisableFilter(*socket_);
           if (!use_filter_) {
             cout << "  * Filter Off" << endl;
           } else {
             cout << "  * Failed" << endl;
           }
         }
         StartRunning(*socket_);
       }, "Filter ON/OFF"});
#endif

#ifdef CES_SPECIAL
  pcviz_->AddCtrlShortcut(
      {"c",
       [this](){
         ces_static_scene_->SetShow(!ces_static_scene_->Showing());
       },
       "Display CES Scene"});
#endif

  pcviz_->PrintShortcuts();
}
