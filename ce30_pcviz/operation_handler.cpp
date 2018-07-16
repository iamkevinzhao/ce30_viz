#include "operation_handler.h"
#include <iostream>
#include "static_view.h"
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl::visualization;
using namespace std::chrono;

namespace ce30_pcviz {
OperationHandler::OperationHandler(shared_ptr<PCLVisualizer> viz)
  : viz_(viz),
    double_tapped_(false),
    point_pick_on_(false)
{
  viz_->registerKeyboardCallback(
        boost::bind(&OperationHandler::HandleKeyboardEvent, this, _1));
  viz_->registerMouseCallback(
        boost::bind(&OperationHandler::HandleMouseEvent, this, _1));
  viz_->registerPointPickingCallback(
        boost::bind(&OperationHandler::HandlePointPickingEvent, this, _1));
  aerial_view_.reset(
      new StaticView(viz_, -10.0f, 0.0f, 5.0f, 10.0f, 0.0f, 0.0f));
  vertical_view_.reset(
      new StaticView(viz_, 15.0f, 0.0f, 50.0f, 16.0f, 0.0f, 0.0f));
  AddShortcut(
      {"1", [this](){aerial_view_->Change();}, "Switch to Aerial View"});
  AddShortcut(
      {"2", [this](){vertical_view_->Change();}, "Switch to Vertical View"});
  AddShortcut(
      {"p",
       [this](){
         point_pick_on_ = !point_pick_on_;
         if (point_pick_on_) {
           std::cout << "Point Picking Mode On" << std::endl;
         } else {
           std::cout << "Point Picking Mode Off" << std::endl;
           if (point_picking_mode_off_callback_) {
             point_picking_mode_off_callback_();
           }
         }
       }, "Switch Point Picking Mode"});
}

void OperationHandler::HandleMouseEvent(const MouseEvent &event) {

}

void OperationHandler::HandlePointPickingEvent(
    const PointPickingEvent &event) {
  if (!point_pick_on_) {
    return;
  }
  float x, y, z;
  event.getPoint(x, y, z);
  if (point_picked_callback_) {
    point_picked_callback_(x, y, z);
  }
//  std::cout
//      << "Picked Point: "
//      << std::setprecision(2)
//      << x << ", " << y << ", " << z << std::endl;
}

void OperationHandler::HandleKeyboardEvent(const KeyboardEvent &event) {
  if (double_tapped_) {
    if (event.isCtrlPressed() && IsNumKey(event.getKeySym())) {
      // Ctrl + NUM
    } else {
      double_tapped_ = !double_tapped_;
      return;
    }
  }

  for (auto& shortcut : ctrl_shortcuts_) {
    if (event.isCtrlPressed() && event.getKeySym() == shortcut.key) {
      shortcut.callback();
    }
  }

  double_tapped_ = !double_tapped_;
}

void OperationHandler::AddShortcut(const CtrlShortcut &shortcut) {
  ctrl_shortcuts_.push_back(shortcut);
}

std::vector<CtrlShortcut> OperationHandler::GetAllCtrlShortcuts() {
  return ctrl_shortcuts_;
}

vector<pair<string, string>> OperationHandler::CtrlShortcutMap() {
  vector<pair<string, string>> map;
  for (auto& shortcut : ctrl_shortcuts_) {
    map.push_back({shortcut.key, shortcut.description});
  }
  return map;
}

void OperationHandler::PrintShortcuts() {
  cout << "Shortcuts:" << endl;
  for (auto& shortcut : ctrl_shortcuts_) {
    cout
        << "  * Ctrl+'" << shortcut.key
        << "' -- " << shortcut.description << endl;
  }
}

bool OperationHandler::IsNumKey(const string &key) {
  if (key >= "0" && key <= "9") {
    return true;
  }
  return false;
}

void OperationHandler::RegisterPointPickedCallback(
    std::function<void (float, float, float)> callback) {
  point_picked_callback_ = callback;
}

void OperationHandler::RegisterPointPickingModeOffCallback(
    std::function<void ()> callback) {
  point_picking_mode_off_callback_ = callback;
}

void OperationHandler::UseAerialView() {
  aerial_view_->Change();
}

void OperationHandler::UseVerticalView() {
  vertical_view_->Change();
}
}
