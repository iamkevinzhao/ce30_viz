#pragma once
#ifndef CE30_VIZ_CE30_PCVIZ_H
#define CE30_VIZ_CE30_PCVIZ_H

#include <ce30_pcviz/config.h>
#include "export.h"
#include <string>

namespace ce30_pcviz {
class API PointCloudViz {
public:
  PointCloudViz();
  static std::string Version();
};
}

#endif
