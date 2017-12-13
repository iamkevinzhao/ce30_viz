#ifndef OPERATION_HANDLER_H
#define OPERATION_HANDLER_H

#include <memory>
#include <pcl/visualization/pcl_visualizer.h>
#include "static_view.h"
#include <chrono>
#include <unordered_map>

namespace ce30_pcviz {
struct CtrlShortcut {
  std::string key;
  std::function<void()> callback;
  std::string description;
};

class OperationHandler
{
public:
  OperationHandler(std::shared_ptr<pcl::visualization::PCLVisualizer> viz);
  void UseAerialView();
  void UseVerticalView();
  void AddShortcut(const CtrlShortcut& shortcut);
  std::vector<std::pair<std::string, std::string>> CtrlShortcutMap();
  void PrintShortcuts();
protected:
  void HandleMouseEvent(const pcl::visualization::MouseEvent& event);
  void HandleKeyboardEvent(const pcl::visualization::KeyboardEvent& event);
private:
  std::shared_ptr<pcl::visualization::PCLVisualizer> viz_;
  std::unique_ptr<StaticView> aerial_view_;
  std::unique_ptr<StaticView> vertical_view_;
  std::vector<CtrlShortcut> ctrl_shortcuts_;
  std::chrono::high_resolution_clock::time_point last_tap_time_;
};
}

#endif // OPERATION_HANDLER_H
