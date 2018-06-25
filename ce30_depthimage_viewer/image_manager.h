#ifndef IMAGE_MANAGER_H
#define IMAGE_MANAGER_H

#include <QPixmap>
#include <QImage>
#include <memory>
#include <mutex>
#include "gray_image.h"
#include <ce30_drivers/packet.h>

class ImageManager
{
public:
  ImageManager();
  virtual ~ImageManager();
  std::unique_ptr<QPixmap> GetPixmap();
  std::unique_ptr<GrayImage> GetGrayImage();
  void SetGrayImage(std::unique_ptr<GrayImage> image);
  void SetGrayImageFromScan(const ce30_driver::Scan& scan);
private:
  std::mutex image_mutex_;
  std::unique_ptr<GrayImage> image_;
};

#endif // IMAGE_MANAGER_H
