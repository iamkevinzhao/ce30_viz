#include <QCoreApplication>
#include "point_cloud_viewer.h"
#include <QTimer>

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  PointCloudViewer viewer;
  return app.exec();
}
