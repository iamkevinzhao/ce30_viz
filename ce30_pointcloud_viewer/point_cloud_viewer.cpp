#include "point_cloud_viewer.h"
#include <iostream>
#include <ce30_pcviz/ce30_pcviz.h>
#include <ce30_driver/utils.h>
#include <QCoreApplication>
#include <QThread>
#include <QTime>
#include <QDir>
#include <QElapsedTimer>
#include "grey_image.h"
#include <ce30_pcviz/world_scene_x.h>

using namespace std;
using namespace ce30_pcviz;
using namespace ce30_driver;

PointCloudViewer::PointCloudViewer()
  : vertical_stretch_mode_(true),
    save_pcd_(false),
    use_filter_(false),
    kill_signal_(false)
{
  startTimer(0);
  ce30_pcviz::Point::SetXRange(Channel::DistanceMin(), Channel::DistanceMax());
  control_panel_.reset(new ControlPanelWidget);
#ifdef SUPPORT_CHANNEL_TYPE_FEATURE
  channel_type_widgets_.reset(new ChannelTypeWidgets(control_panel_->Widget()));
  control_panel_->AppendWidgets(channel_type_widgets_->GetAllWidgets());
#endif
  connect(
      this, SIGNAL(ShowControlPanel(std::vector<ce30_pcviz::CtrlShortcut>)),
      control_panel_.get(),
      SLOT(OnShow(std::vector<ce30_pcviz::CtrlShortcut>)));
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
  emit HideGrayImage();
#ifdef SUPPORT_GRAY_OUTPUT_MODE
  if (!DisableGrayOutput(socket)) {
    cerr << "Unable to Switch off Gray Output" << endl;
    return ExitCode::switch_gray_output_failure;
  }
#endif
  if (!DeviceConfig::Configure(socket)) {
    cerr << "Failed to Configure CE30" << endl;
    return ExitCode::configure_ce30_failure;
  }
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
    if (!thread_) {
      thread_.reset(
          new std::thread(bind(&PointCloudViewer::PacketReceiveThread, this)));
    }
  }
  if (!pcviz_) {
    pcviz_.reset(new PointCloudViz);
#if ON_DEVEL
    pcviz_->UpdateWorldScene(
        std::shared_ptr<WorldSceneX>(new WorldSceneX(pcviz_->GetPCLViz())));
#endif
    OnPCVizInitialized();
  }
  if (pcviz_->Closed()) {
     QCoreApplication::exit((int)ExitCode::normal_exit);
    return;
  }

  static Scan scan;
  static Scan grey_scan;
  std::unique_lock<std::mutex> lock(scan_mutex_);
  condition_.wait(lock);
  if (scan_.Ready()) {
    scan = scan_;
  }
  if (grey_scan_.Ready()) {
    grey_scan = grey_scan_;
  }
  lock.unlock();

#ifdef SUPPORT_CHANNEL_TYPE_FEATURE
  channel_type_widgets_->Update();
#endif

  if (scan.Ready()) {
    UpdatePointCloudDisplay(
        scan, *pcviz_, vertical_stretch_mode_, save_pcd_);
  }
  if (grey_scan.Ready()) {
    UpdateGreyImageDisplay(grey_scan);
  }
}

void PointCloudViewer::UpdatePointCloudDisplay(
    const Scan &scan,
    PointCloudViz &viz,
    const bool& vsmode,
    const bool& save_pcd) {
  ce30_pcviz::PointCloud cloud;
  cloud.Reserve(scan.Width() * scan.Height());
  for (int x = 0; x < scan.Width(); ++x) {
    for (int y = 0; y < scan.Height(); ++y) {
      auto channel = scan.at(x, y);
#ifdef SUPPORT_CHANNEL_TYPE_FEATURE
      if (!(channel_type_widgets_->IsChannelTypeChecked(channel))) {
        continue;
      }
#endif
      ce30_driver::Point p = channel.point();
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
  static const QString data_dir_name = "data";
  if (save_pcd) {
    if (!QDir(data_dir_name).exists()) {
      QDir::current().mkdir(data_dir_name);
    }
    viz.SavePCD(
        data_dir_name.toStdString() + "/" +
        QTime::currentTime().toString("hh_mm_ss_zzz").toStdString() +
        ".pcd", cloud);
  }
//  bool save_images = save_pcd;
  bool save_images = false;
  if (save_images) {
    if (!QDir(data_dir_name).exists()) {
      QDir::current().mkdir(data_dir_name);
    }
    ce30_driver::SaveImages(
        data_dir_name.toStdString() + "/" +
        QTime::currentTime().toString("hh_mm_ss_zzz").toStdString() +
        ".txt", scan);
  }
}

void PointCloudViewer::UpdateGreyImageDisplay(const Scan &scan) {
//  const auto min = ce30_driver::Channel::GreyValueMin();
//  const auto max = ce30_driver::Channel::GreyValueMax();
  const auto width = ce30_driver::Scan::Width();
  const auto height = ce30_driver::Scan::Height();

  const unsigned short min = ce30_driver::Channel::GreyValueMin();
  unsigned short max = min;

  for (int w = 0; w < width; ++w) {
    for (int h = 0; h < height; ++h) {
      auto value = scan.at(w, h).grey_value;
      if (value > max) {
        max = value;
      }
    }
  }
  if (max <= min) {
    max = ce30_driver::Channel::GreyValueMax();
  }
  std::shared_ptr<GrayImage> image(
      new GrayImage(width, height, min, max));
  for (int w = 0; w < width; ++w) {
    for (int h = 0; h < height; ++h) {
      auto value = scan.at(width - w - 1, h).grey_value;
      if (value < min) {
        value = min;
      }
      if (value > max) {
        value = max;
      }
      // image->SetPixel({w, h}, scan.at(w, h).grey_value);
      image->SetPixel({w, h}, value);
      // std::cout << scan.at(w, h).grey_value << std::endl;
    }
  }
  emit UpdateImage(image);
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
    static Scan grey_scan;
    while (!scan.Ready() && !grey_scan.Ready()) {
      if (GetPacket(packet, *socket_, true)) {
        auto parsed = packet.Parse();
        if (parsed) {
          if (parsed->grey_image) {
            grey_scan.AddColumnsFromPacket(*parsed);
          } else {
            scan.AddColumnsFromPacket(*parsed);
          }
        } else {
          cerr << "Error parsing package." << endl;
        }
      } else {
        cerr << "Error getting package." << endl;
      }
    }
    unique_lock<mutex> lock(scan_mutex_);
    if (grey_scan.Ready()) {
      grey_scan_ = grey_scan;
    }
    if (scan.Ready()) {
      scan_ = scan;
    }
    condition_.notify_all();
    lock.unlock();
    if (grey_scan.Ready()) {
      grey_scan.Reset();
    }
    if (scan.Ready()) {
      scan.Reset();
    }
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
  pcviz_->AddCtrlShortcut(
      {"d",
       [this](){
         pcviz_->ClusterModeOn(!pcviz_->IsClusterModeOn());
       }, "Clustering Mode"});

#ifdef SUPPORT_GRAY_OUTPUT_MODE
  pcviz_->AddCtrlShortcut(
      {"g",
       [this](){
         emit ShowGrayImage();
         EnableGrayOutput(*socket_);
       }, "Switch on Gray Image"});
#endif

  auto func = [this](){
    emit ShowControlPanel(pcviz_->GetAllCtrlShortcuts());
  };
  pcviz_->AddCtrlShortcut(
      {"c", func, "Show Control Panel"});
  func();

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
