#ifndef CES_STATIC_SCENE_H
#define CES_STATIC_SCENE_H

#include <ce30_pcviz/scene.h>

class CESStaticScene : public ce30_pcviz::Scene
{
public:
  CESStaticScene();
  void SetShow(const bool& show);
  bool Showing();
  void Update() override;
private:
  static std::vector<std::string> DrawScene(
      pcl::visualization::PCLVisualizer& viz);
  static void EraseScene(
      pcl::visualization::PCLVisualizer& viz,
      const std::vector<std::string> ids);
  static std::string GetLineID(const int& id);
  bool showing_;
  bool last_showing_;
  std::vector<std::string> scene_ids_;
};

#endif // CES_STATIC_SCENE_H
