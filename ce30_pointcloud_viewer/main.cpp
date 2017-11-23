
#include "ce30_pcviz/ce30_pcviz.h"
#include <thread>

int main(int argc, char* argv[])
{
  ce30_pcviz();
  std::this_thread::sleep_for(std::chrono::seconds(10));
  return 0;
}
