#ifndef OPERATION_HANDLER_H
#define OPERATION_HANDLER_H

#include <memory>
#include <pcl/visualization/pcl_visualizer.h>
#include "static_view.h"

namespace ce30_pcviz {
class OperationHandler
{
public:
  OperationHandler(std::shared_ptr<pcl::visualization::PCLVisualizer> viz);
  void ApplyInitialView();
protected:
  void HandleMouseEvent(const pcl::visualization::MouseEvent& event);
  void HandleKeyboardEvent(const pcl::visualization::KeyboardEvent& event);
private:
  std::shared_ptr<pcl::visualization::PCLVisualizer> viz_;
  std::unique_ptr<StaticView> initial_view_;
};
}

#endif // OPERATION_HANDLER_H
