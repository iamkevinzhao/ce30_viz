#include <QApplication>
#include "point_cloud_viewer.h"
#include <QTimer>
#include <QObject>

#ifdef FAKE_POINTCLOUD
#include "fake_point_cloud_viewer.h"
#endif

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
#ifdef FAKE_POINTCLOUD
  FakePointCloudViewer viewer;
#else
  visualizer::PointCloudViewer viewer;
#endif
  return app.exec();
}
