#ifndef POINT_CLOUD_VIEWER_H
#define POINT_CLOUD_VIEWER_H

#include <QObject>
#include <memory>
#include <ce30_drivers/ce30_x_driver.h>
#include <ce30_pcviz/ce30_pcviz.h>
#include <QCoreApplication>
#include "exit_code.h"
#include <thread>
#include <mutex>
#include <QImage>

namespace ce30_pcviz {
  class ControlPanelWidget;
}

namespace visualizer {
class PointCloudViewer : public QObject
{
  Q_OBJECT
 public:
  PointCloudViewer();
  ~PointCloudViewer();
 signals:
  void UpdateImage(std::shared_ptr<QImage> image);
  void ShowControlPanel(std::vector<ce30_pcviz::CtrlShortcut> shortcuts);
 protected:
  void timerEvent(QTimerEvent* event);
 private:
  void PacketReceiveThread();
  void OnPCVizInitialized();
  static ExitCode ConnectOrExit(ce30_drivers::UDPSocket& socket);
  static void UpdatePointCloudDisplay(
      const ce30_x::Scan& scan,
      ce30_pcviz::PointCloudViz& viz,
      const bool& vsmode,
      const bool& save_pcd);
  std::shared_ptr<ce30_drivers::UDPSocket> socket_;
  std::unique_ptr<ce30_pcviz::PointCloudViz> pcviz_;
  bool vertical_stretch_mode_;
  bool save_pcd_;
  bool use_filter_;
#ifdef CES_SPECIAL
  std::shared_ptr<CESStaticScene> ces_static_scene_;
#endif
  std::unique_ptr<std::thread> thread_;
  std::mutex scan_mutex_;
  ce30_x::Scan scan_;
//  ce30_x::Scan grey_scan_;
  std::mutex signal_mutex_;
  bool kill_signal_;
  std::condition_variable condition_;
  std::shared_ptr<ce30_pcviz::ControlPanelWidget> control_panel_;
};
} // namespace visualizer
#endif // POINT_CLOUD_VIEWER_H
