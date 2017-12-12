#ifndef POINT_CLOUD_VIEWER_H
#define POINT_CLOUD_VIEWER_H

#include <QObject>
#include <memory>
#include <ce30_driver/ce30_driver.h>
#include <ce30_pcviz/ce30_pcviz.h>
#include <QCoreApplication>
#include "exit_code.h"

class PointCloudViewer : public QObject
{
public:
  PointCloudViewer();
  ~PointCloudViewer();
protected:
  void timerEvent(QTimerEvent* event);
private:
  void OnPCVizInitialized();
  static ExitCode ConnectOrExit(ce30_driver::UDPSocket& socket);
  static void UpdatePointCloudDisplay(
      const ce30_driver::Scan& scan,
      ce30_pcviz::PointCloudViz& viz,
      const bool& vsmode);
  std::unique_ptr<ce30_driver::UDPSocket> socket_;
  std::unique_ptr<ce30_pcviz::PointCloudViz> pcviz_;
  ce30_driver::Scan scan_;
  bool vertical_stretch_mode_;
};

#endif // POINT_CLOUD_VIEWER_H
