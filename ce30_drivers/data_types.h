#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <vector>
#include "ce30_drivers_export.h"

namespace ce30_drivers {
/**
 * @brief 3D point
 */
struct CE30_DRIVERS_API Point {
  /**
   * @brief constructor
   */
  Point();
  /**
   * @brief constructor
   * @param x x in meters
   * @param y y in meters
   * @param z z in meters
   */
  Point(const float& x, const float& y, const float& z);
  /**
   * @brief x in meters
   */
  float x;
  /**
   * @brief y in meters
   */
  float y;
  /**
   * @brief z in meters
   */
  float z;
};

/**
 * @brief point cloud
 */
struct CE30_DRIVERS_API PointCloud {
  /**
   * @brief points of this cloud
   */
  std::vector<Point> points;
};
}
#endif // DATA_TYPES_H
