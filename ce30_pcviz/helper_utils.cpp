#include "helper_utils.h"
#include <sstream>
#include <iomanip>

namespace ce30_pcviz {
std::string ToCoordinateString(const float& x, const float& y, const float& z) {
  std::stringstream ss;
  ss << std::setprecision(2) << x << ", " << y << ", " << z;
  return ss.str();
}
} // namespace
