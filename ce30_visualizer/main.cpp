#include <QApplication>
#include "point_cloud_viewer.h"
#include <QTimer>
#include <QObject>

#ifdef FAKE_POINTCLOUD
#include "fake_point_cloud_viewer.h"
#endif

const std::string kCE30VisualizerVersion = "v1.0.0";

int main(int argc, char *argv[])
{
  std::cout << "Software Version: " << kCE30VisualizerVersion << std::endl;

  QApplication app(argc, argv);
#ifdef FAKE_POINTCLOUD
  FakePointCloudViewer viewer;
#else
  visualizer::PointCloudViewer viewer;
#endif
  return app.exec();
}
