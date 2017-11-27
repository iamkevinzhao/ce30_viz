#include "operation_handler.h"
#include <iostream>
#include "static_view.h"

using namespace std;
using namespace pcl::visualization;

namespace ce30_pcviz {
OperationHandler::OperationHandler(shared_ptr<PCLVisualizer> viz) : viz_(viz)
{
  viz_->registerKeyboardCallback(
        boost::bind(&OperationHandler::HandleKeyboardEvent, this, _1));
  viz_->registerMouseCallback(
        boost::bind(&OperationHandler::HandleMouseEvent, this, _1));
  initial_view_.reset(
      new StaticView(viz_, -10.0f, 0.0f, 5.0f, 10.0f, 0.0f, 0.0f));
}

void OperationHandler::HandleMouseEvent(const MouseEvent &event) {

}

void OperationHandler::HandleKeyboardEvent(const KeyboardEvent &event) {
  if (event.isCtrlPressed() && event.getKeySym() == "1") {
    ApplyInitialView();
  }
}

void OperationHandler::ApplyInitialView() {
  initial_view_->Change();
}
}
