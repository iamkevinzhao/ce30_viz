#ifndef POINT_CLOUD_VIEWER_H
#define POINT_CLOUD_VIEWER_H

#include <QObject>
#include <memory>
#include <ce30_drivers/ce30_d_driver.h>
#include <ce30_pcviz/ce30_pcviz.h>
#include <QCoreApplication>
#include "exit_code.h"
#include <thread>
#include <mutex>
#include <QImage>

#ifdef CES_SPECIAL
#include "ces_special/ces_static_scene.h"
#endif

class PointCloudViewer : public QObject
{
  Q_OBJECT
 public:
  PointCloudViewer();
  ~PointCloudViewer();
 signals:
  void UpdateImage(std::shared_ptr<QImage> image);
 protected:
  void timerEvent(QTimerEvent* event);
 private:
  void PacketReceiveThread();
  void OnPCVizInitialized();
  static ExitCode ConnectOrExit(ce30_drivers::UDPSocket& socket);
  static void UpdatePointCloudDisplay(
      const ce30_d::Scan& scan,
      ce30_pcviz::PointCloudViz& viz,
      const bool& vsmode,
      const bool& save_pcd);
  void UpdateGreyImageDisplay(
      const ce30_d::Scan& scan);
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
  ce30_d::Scan scan_;
  ce30_d::Scan grey_scan_;
  std::mutex signal_mutex_;
  bool kill_signal_;
  std::condition_variable condition_;
};

#endif // POINT_CLOUD_VIEWER_H