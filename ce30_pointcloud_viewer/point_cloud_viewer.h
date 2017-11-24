#ifndef POINT_CLOUD_VIEWER_H
#define POINT_CLOUD_VIEWER_H

#include <QObject>
#include <memory>
#include <ce30_driver/udp_socket.h>
#include <QCoreApplication>
#include "exit_code.h"

class PointCloudViewer : public QObject
{
public:
  PointCloudViewer();
  ~PointCloudViewer();
  bool Init();
protected:
  void timerEvent(QTimerEvent* event);
private:
  std::unique_ptr<ce30_driver::UDPSocket> socket_;
  ExitCode exit_code_;
};

#endif // POINT_CLOUD_VIEWER_H
