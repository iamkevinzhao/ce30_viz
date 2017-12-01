#include "gray_image.h"
#include <QDebug>

using namespace std;

const int GrayImage::kHighlightDelta = 50;
const int GrayImage::kIndexMax = 254;
const int GrayImage::kIndexMin = 0;

GrayImage::GrayImage(const GrayImage& image) {
  *this = image;
}

GrayImage::GrayImage(const QSize& size) : GrayImage(size, kIndexMin, kIndexMax) {}

GrayImage::GrayImage(
    const int& width, const int& height, const float& from, const float& to)
  : GrayImage(QSize(width, height), from, to)
{
}

GrayImage::GrayImage(const QSize& size, const float& from, const float& to)
  : QImage(size, Format_Indexed8),
    from_(from),
    to_(to)
{
  setColorTable(GenerateColorTable());
  fill(0);
}

QVector<QRgb> GrayImage::GenerateColorTable() {
  QVector<QRgb> table;
  table.reserve(kIndexMax - kIndexMin + 1);
  for (int i = kIndexMin; i <= kIndexMax; ++i) {
    table.push_back(qRgb(i, i, i));
  }
  return table;
}

bool GrayImage::IsRegionValid(const QRect &region) const {
  auto w = width();
  auto h = height();

  auto x1 = region.left();
  auto x2 = region.right();
  auto y1 = region.top();
  auto y2 = region.bottom();

  if (w == 0 || h == 0) {
    return false;
  }
  if (x1 < 0 || x1 >= w) {
    return false;
  }
  if (x2 < 0 || x2 >= w) {
    return false;
  }
  if (x1 > x2) {
    return false;
  }
  if (y1 < 0 || y1 >= h) {
    return false;
  }
  if (y2 < 0 || y2 >= h) {
    return false;
  }
  if (y1 > y2) {
    return false;
  }
  return true;
}

unique_ptr<GrayImage> GrayImage::Chop(const QRect& region) const {
  if (!IsRegionValid(region)) {
    return unique_ptr<GrayImage>();
  }

  int new_w = region.right() - region.left() + 1;
  int new_h = region.bottom() - region.top() + 1;
  auto result = unique_ptr<GrayImage>(new GrayImage(QSize(new_w, new_h)));
  for (int x = 0; x < new_w; ++x) {
    for (int y = 0; y < new_h; ++y) {
      result->setPixel(x, y, pixelIndex(x + region.left(), y + region.top()));
    }
  }
  return result;
}

unique_ptr<GrayImage> GrayImage::Highlight(const QRect &region) const {
  if (!IsRegionValid(region)) {
    return unique_ptr<GrayImage>();
  }

  int new_w = region.right() - region.left() + 1;
  int new_h = region.bottom() - region.top() + 1;
  auto result = unique_ptr<GrayImage>(new GrayImage(*this));
  for (int x = region.left(); x <= region.right(); ++x) {
    for (int y = region.top(); y < region.bottom(); ++y) {
      auto index = pixelIndex(x, y);
      if ((index + kHighlightDelta) > kIndexMax) {
        index -= kHighlightDelta;
      } else {
        index += kHighlightDelta;
      }
      result->setPixel(x, y, index);
    }
  }
  return result;
}
