#include <QCoreApplication>
#include "point_cloud_viewer.h"
#include <QTimer>

#ifdef FAKE_POINTCLOUD
#include "fake_point_cloud_viewer.h"
#endif

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
#ifdef FAKE_POINTCLOUD
  FakePointCloudViewer viewer;
#else
  PointCloudViewer viewer;
#endif
  return app.exec();
}
