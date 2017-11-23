#include <QCoreApplication>
#include "ce30_pcviz/ce30_pcviz.h"

int main(int argc, char *argv[])
{
  QCoreApplication a(argc, argv);

  ce30_pcviz();

  return a.exec();
}
