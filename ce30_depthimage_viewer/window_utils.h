#ifndef WINDOW_UTILS_H
#define WINDOW_UTILS_H

#include <QWidget>

class WindowUtils
{
public:
  WindowUtils();
  static void MoveToCenter(QWidget* widget);
  static QString WarningMessageBoxTitle();
};

#endif // WINDOW_UTILS_H
