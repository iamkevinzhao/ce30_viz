#include "point_cloud_viewer.h"
#include <iostream>
#include <QCoreApplication>
#include <QThread>
#include <QTime>
#include <QDir>
#include <QElapsedTimer>
#include <ce30_pcviz/gray_image.h>

#include <ce30_drivers/ce30_d_driver.h>
#include <chrono>

using namespace std;
using namespace ce30_pcviz;
// using namespace ce30_drivers;
using namespace ce30_x;

namespace visualizer {
PointCloudViewer::PointCloudViewer()
  : vertical_stretch_mode_(true),
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
  // StopRunning(*socket_);
}

ExitCode PointCloudViewer::ConnectOrExit(ce30_drivers::UDPSocket& socket) {
  if (!Connect(socket)) {
    cerr << "Unable to Connect Device!" << endl;
    return ExitCode::device_connection_failure;
  }
//  string device_version;
//  if (!ce30_d::GetVersion(device_version, socket)) {
//    cerr << "Unable to Retrieve CE30 Device Version" << endl;
//    return ExitCode::retrieve_ce30_version_failure;
//  }
//  cout << "CE30 Version: " << device_version << endl;
//  if (!ce30_d::StartRunning(socket)) {
//    cerr << "Unable to Start CE30" << endl;
//    return ExitCode::start_ce30_failure;
//  }
  return ExitCode::no_exit;
}

void PointCloudViewer::timerEvent(QTimerEvent *event) {
  if (!socket_) {
    socket_.reset(new ce30_drivers::UDPSocket("192.168.0.2", 2468));
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
  // condition_.wait(lock);
  if (condition_.wait_for(lock, std::chrono::milliseconds(100)) ==
      std::cv_status::no_timeout) {
    if (scan_.Ready()) {
      scan = scan_;
    }
    lock.unlock();

    if (scan.Ready()) {
      UpdatePointCloudDisplay(
          scan, *pcviz_, vertical_stretch_mode_, save_pcd_);
    }
  } else {
    lock.unlock();
    pcviz_->SpinOnce();
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
//  for (int x = 0; x < scan.Width(); ++x) {
//    for (int y = 0; y < scan.Height(); ++y) {
//      ce30_drivers::Point p = scan.at(x, y).point();
//      if (vsmode) {
//        p.z = (scan.Height() - y) * 0.1f;
//      }
//      if (sqrt(p.x * p.x + p.y * p.y) < 0.1f) {
//        continue;
//      }
//      cloud.push_back(ce30_pcviz::Point(p.x, p.y, p.z));
//    }
//  }
  auto channels = scan.GetChannels();
  constexpr float kIgnoreDistanceMin = 10.0f;
  constexpr float kIgnoreDistanceMax = 200.0f;
  float dist_max = kIgnoreDistanceMin;
  float dist_min = kIgnoreDistanceMax;

  float dist;
  for (auto& channel : channels) {
    dist = channel.distance;
    if (dist > kIgnoreDistanceMax || dist < kIgnoreDistanceMin) {
      continue;
    }
    if (dist > dist_max) {
      dist_max = dist;
    }
    if (dist < dist_min) {
      dist_min = dist;
    }
  }
  ce30_pcviz::Point::SetXRange(dist_min, dist_max);

  cloud.Reserve(channels.size());
  for (auto& channel : channels) {
    dist = channel.distance;
    if (dist > kIgnoreDistanceMax || dist < kIgnoreDistanceMin) {
      continue;
    }
    cloud.push_back(ce30_pcviz::Point(dist, channel.x, channel.y));
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

    static QElapsedTimer timer;
    static bool init = false;
    if (!init) {
      timer.start();
      init = true;
    }
    static int cnt = 0;

    static Packet packet;
    static Scan scan;
    while (!scan.Ready() && !kill_signal_) {
      if (GetPacket(packet, *socket_, true)) {
//        ++cnt;
//        auto elapsed = timer.elapsed();
//        if (elapsed > 1000) {
//          qDebug() << "Frequency: " <<  cnt * 1.0f / (elapsed / 1000.0f);
//          timer.restart();
//          cnt = 0;
//        }
        auto parsed = packet.Parse();
        if (parsed) {
          scan.AddFromPacket(*parsed);
        } else {
          // cerr << "Error parsing package." << endl;
        }
      } else {
        cerr << "Error getting package." << endl;
      }
    }
    unique_lock<mutex> lock(scan_mutex_);
    if (scan.Ready()) {
      scan_ = scan;
    }
    condition_.notify_all();
    lock.unlock();
    if (scan.Ready()) {
      scan.Reset();
    }
  }
}

void PointCloudViewer::OnPCVizInitialized() {
  pcviz_->AddCtrlShortcut(
      {"t",
       [this](){
         cout << "Laser ON" <<endl;
         LaserOnSendPacket packet;
         SendPacket(packet, *socket_, true);
       },
       "Laser ON"});
  pcviz_->AddCtrlShortcut(
      {"l",
       [this](){
          cout << "Laser OFF" << endl;
         LaserOffSendPacket packet;
         SendPacket(packet, *socket_, true);
       }, "Laser OFF "});
  pcviz_->AddCtrlShortcut(
      {"d",
       [this](){
          cout << "MEMS ON" << endl;
         MEMSOn packet;
         SendPacket(packet, *socket_, true);
       }, "MEMS ON"});
  pcviz_->AddCtrlShortcut(
      {"u",
       [this](){
          cout << "MEMS OFF" << endl;
         MEMSOff packet;
         SendPacket(packet, *socket_, true);
       }, "MEMS OFF"});

  pcviz_->PrintShortcuts();
}

}
