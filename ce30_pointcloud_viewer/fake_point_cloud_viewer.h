#ifndef FAKE_POINT_CLOUD_VIEWER_H
#define FAKE_POINT_CLOUD_VIEWER_H

#include <QObject>
#include <QTimerEvent>
#include <ce30_pcviz/ce30_pcviz.h>

class FakePointCloudViewer : public QObject
{
public:
  FakePointCloudViewer();
protected:
  void timerEvent(QTimerEvent* event);
private:
  void ExecuteCycle();
  int timer_id_;
  ce30_pcviz::PointCloudViz viz_;
};

#endif // FAKE_POINT_CLOUD_VIEWER_H
