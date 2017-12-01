#include "image_manager.h"
#include <QDebug>
#include <iostream>

using namespace std;
using namespace ce30_driver;

ImageManager::ImageManager()
{
}

ImageManager::~ImageManager() {}

unique_ptr<QPixmap> ImageManager::GetPixmap() {
  if (!image_) {
    return unique_ptr<QPixmap>();
  }
  unique_lock<mutex> lock(image_mutex_);
  auto cache = *image_;
  lock.unlock();

  unique_ptr<QPixmap> pixmap(new QPixmap);
  *pixmap = QPixmap::fromImage(cache);
  return std::move(pixmap);
}

unique_ptr<GrayImage> ImageManager::GetGrayImage() {
  unique_lock<mutex> lock(image_mutex_);
  unique_ptr<GrayImage> cache(new GrayImage(*image_));
  lock.unlock();
  return cache;
}

void ImageManager::SetGrayImage(unique_ptr<GrayImage> image) {
  unique_lock<mutex> lock(image_mutex_);
  image_ = std::move(image);
  lock.unlock();
}

void ImageManager::SetGrayImageFromScan(const Scan &scan) {
  unique_ptr<GrayImage> image(
      new GrayImage(
          Scan::Width(), Scan::Height(),
          Scan::DistanceMin(), Scan::DistanceMax()));
  for (int x = 0; x < Scan::Width(); ++x) {
    for (int y = 0; y < Scan::Height(); ++y) {
      image->SetPixel(QPoint(x, y), scan.at(x, y).distance);
    }
  }
  return SetGrayImage(std::move(image));
}
