#include "point_cloud_viewer.h"
#include <iostream>
#include <ce30_pcviz/ce30_pcviz.h>
#include <ce30_driver/utils.h>
#include <QCoreApplication>
#include <QThread>
#include <QTime>
#include <QDir>
#include <QElapsedTimer>

using namespace std;
using namespace ce30_pcviz;
using namespace ce30_driver;

PointCloudViewer::PointCloudViewer()
  : vertical_stretch_mode_(false),
    save_pcd_(false),
    use_filter_(false),
    kill_signal_(false)
{
  startTimer(0);
  ce30_pcviz::Point::SetXRange(Channel::DistanceMin(), Channel::DistanceMax());
}

PointCloudViewer::~PointCloudViewer() {
  signal_mutex_.lock();
  kill_signal_ = true;
  signal_mutex_.unlock();

  if (thread_ && thread_->joinable()) {
    thread_->join();
  }
  StopRunning(*socket_);
}

ExitCode PointCloudViewer::ConnectOrExit(UDPSocket& socket) {
//  if (!Connect(socket)) {
//    cerr << "Unable to Connect Device!" << endl;
//    return ExitCode::device_connection_failure;
//  }
//  string device_version;
//  if (!GetVersion(device_version, socket)) {
//    cerr << "Unable to Retrieve CE30 Device Version" << endl;
//    return ExitCode::retrieve_ce30_version_failure;
//  }
//  cout << "CE30 Version: " << device_version << endl;
//  if (!StartRunning(socket)) {
//    cerr << "Unable to Start CE30" << endl;
//    return ExitCode::start_ce30_failure;
//  }
  return ExitCode::no_exit;
}

void PointCloudViewer::timerEvent(QTimerEvent *event) {
  if (!socket_) {
    ifstream is("ip.txt");
    string ip, port;
    if (!is.is_open()) {
      socket_.reset(new UDPSocket);
    } else {
      is >> ip >> port;
      cout << "IP: " << ip << " Port: " << port << endl;
      socket_.reset(new UDPSocket(ip, stoi(port)));
    }
    auto ec = ConnectOrExit(*socket_);
    if (ec != ExitCode::no_exit) {
      QThread::sleep(2);
      QCoreApplication::exit((int)ec);
      return;
    }
    if (!thread_) {
      thread_.reset(
          new std::thread(bind(&PointCloudViewer::PacketReceiveThread, this)));
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

  static Scan scan;
  std::unique_lock<std::mutex> lock(scan_mutex_);
  condition_.wait(lock);
  scan = scan_;
  lock.unlock();

  if (scan.Ready()) {
    UpdatePointCloudDisplay(
        scan, *pcviz_, vertical_stretch_mode_, save_pcd_);
  }
}

void PointCloudViewer::UpdatePointCloudDisplay(
    const Scan &scan,
    PointCloudViz &viz,
    const bool& vsmode,
    const bool& save_pcd) {
//  static int cnt = 0;
//  cnt++;
//  static QElapsedTimer timer;
//  if (cnt > 30) {
//    cout << timer.elapsed() << endl;
//    timer.start();
//    cnt = 0;
//  }

  ce30_pcviz::PointCloud cloud;
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

void PointCloudViewer::PacketReceiveThread() {
  while (true) {
    // signal_mutex_.lock();
    auto kill_signal = kill_signal_;
    // signal_mutex_.unlock();
    if (kill_signal) {
      return;
    }

    static Packet packet;
    static Scan scan;
    while (!scan.Ready()) {
      if (GetPacket(packet, *socket_, true)) {
        auto parsed = packet.Parse();
        if (parsed) {
          scan.AddColumnsFromPacket(*parsed);
        }
      }
    }
    scan_mutex_.lock();
    scan_ = scan;
    scan_mutex_.unlock();
    condition_.notify_all();
    scan.Reset();
  }
}

void PointCloudViewer::OnPCVizInitialized() {
#ifdef CES_SPECIAL
  ces_static_scene_.reset(new CESStaticScene);
  pcviz_->UpdateWorldScene(ces_static_scene_);
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
       "Display CES Scene (CES Special)"});
  pcviz_->AddCtrlShortcut(
      {"3",
       [this](){
         ces_static_scene_->ChangeToDefaultViewPoint();
       },
       "Change View Point (CES Special)"});
  pcviz_->AddCtrlShortcut(
      {"u",
       [this](){
         ces_static_scene_->UpdateOffsetDelta(0.0f, 0.0f, 0.05f);
       },
       "Move Scene Up (CES Special)"});
  pcviz_->AddCtrlShortcut(
      {"d",
       [this](){
         ces_static_scene_->UpdateOffsetDelta(0.0f, 0.0f, -0.05f);
       },
       "Move Scene Down (CES Special)"});
  pcviz_->AddCtrlShortcut(
      {"Up",
       [this](){
         ces_static_scene_->UpdateOffsetDelta(0.05f, 0.0f, 0.0f);
       },
       "Move Scene Forward (CES Special)"});
  pcviz_->AddCtrlShortcut(
      {"Down",
       [this](){
         ces_static_scene_->UpdateOffsetDelta(-0.05f, 0.0f, 0.0f);
       },
       "Move Scene Backward (CES Special)"});
  pcviz_->AddCtrlShortcut(
      {"Left",
       [this](){
         ces_static_scene_->UpdateOffsetDelta(0.0f, 0.05f, 0.0f);
       },
       "Move Scene Left (CES Special)"});
  pcviz_->AddCtrlShortcut(
      {"Right",
       [this](){
         ces_static_scene_->UpdateOffsetDelta(0.0f, -0.05f, 0.00f);
       },
       "Move Scene Right (CES Special)"});
#endif

  pcviz_->PrintShortcuts();
}
