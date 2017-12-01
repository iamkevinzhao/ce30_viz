#include "utilities.h"
#include <QStandardPaths>
#include <QDir>

QVector<QString> Utilities::IPSegments(const QString& ip) {
  QVector<QString> segments;
  QString segment;
  for (auto i : ip) {
    if (i == '.') {
      segments.push_back(segment);
      segment.clear();
    } else {
      segment += i;
    }
  }
  if (!segment.isEmpty()) {
    segments.push_back(segment);
  }
  static const int kIPSegmentNum = 4;
  QVector<QString> result;
  for (int i = 0; i < kIPSegmentNum; ++i) {
    if (i < segments.size()) {
      result.push_back(segments[i]);
    } else {
      result.push_back(QString());
    }
  }
  return result;
}

QString Utilities::DocumentsFolderPath() {
  return QStandardPaths::locate(
      QStandardPaths::DocumentsLocation, QString(),
      QStandardPaths::LocateDirectory);
}

QString Utilities::DirFile(const QString dir, const QString file) {
  if (!dir.isEmpty() && dir.endsWith(QDir::separator())) {
    return dir + file;
  }
  return dir + QDir::separator() + file;
}
