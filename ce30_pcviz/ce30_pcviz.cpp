
#include "ce30_pcviz.h"
#include <iostream>
#include <string>

using namespace std;

namespace ce30_pcviz {
PointCloudViz::PointCloudViz() {}

string PointCloudViz::Version() {
  return CE30_VIZ_VERSION_STRING;
}
}
