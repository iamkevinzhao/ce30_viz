#ifndef POINT_CLOUD_VIEWER_H
#define POINT_CLOUD_VIEWER_H

#include <QObject>
#include <memory>
#include <ce30_driver/ce30_driver.h>
#include <ce30_pcviz/ce30_pcviz.h>
#include <QCoreApplication>
#include "exit_code.h"
#include <thread>
#include <mutex>

#ifdef CES_SPECIAL
#include "ces_special/ces_static_scene.h"
#endif

class PointCloudViewer : public QObject
{
public:
  PointCloudViewer();
  ~PointCloudViewer();
protected:
  void timerEvent(QTimerEvent* event);
private:
  void PacketReceiveThread();
  void OnPCVizInitialized();
  static ExitCode ConnectOrExit(ce30_driver::UDPSocket& socket);
  static void UpdatePointCloudDisplay(
      const ce30_driver::Scan& scan,
      ce30_pcviz::PointCloudViz& viz,
      const bool& vsmode,
      const bool& save_pcd);
  std::shared_ptr<ce30_driver::UDPSocket> socket_;
  std::unique_ptr<ce30_pcviz::PointCloudViz> pcviz_;
  bool vertical_stretch_mode_;
  bool save_pcd_;
  bool use_filter_;
#ifdef CES_SPECIAL
  std::shared_ptr<CESStaticScene> ces_static_scene_;
#endif
  std::unique_ptr<std::thread> thread_;
  std::mutex scan_mutex_;
  ce30_driver::Scan scan_;
  std::mutex signal_mutex_;
  bool kill_signal_;
  std::condition_variable condition_;
};

#endif // POINT_CLOUD_VIEWER_H
