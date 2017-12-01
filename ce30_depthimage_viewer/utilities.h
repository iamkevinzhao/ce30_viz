#ifndef UTILITIES_H
#define UTILITIES_H

#include <QString>
#include <QVector>

class Utilities
{
public:
  static QVector<QString> IPSegments(const QString& ip);
  static QString DocumentsFolderPath();
  static QString DirFile(const QString dir, const QString file);
};

#endif // UTILITIES_H
