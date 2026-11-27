#pragma once

#include <memory>
#include <mutex>

#include "krosc/msgs.hpp"
#include "krosc/topic.hpp"
#include "krosc/viz/viz_plugin.hpp"

class QImage;
class QLabel;
class QTimer;
class QWidget;

namespace krosc_ws::viz {

// 웹 viz의 Camera 탭과 같은 D435 color/depth 뷰.
// 토픽 콜백은 네트워크 스레드이므로 최신 프레임만 복사하고,
// QTimer(GUI 스레드)에서 QImage로 변환해 그린다.
class CameraPlugin final : public krosc::viz::VizPlugin {
 public:
  CameraPlugin() : krosc::viz::VizPlugin({"camera", "Camera", "1.0", {}}) {}
  void initialize(krosc::viz::VizHost& host) override;
  void shutdown() override;

 private:
  struct Images {
    std::mutex mutex;
    krosc::msgs::Image color;
    krosc::msgs::Image depth;
    bool color_seen = false;
    bool depth_seen = false;
  };

  void render();
  static QImage to_image(const krosc::msgs::Image& image);
  static QImage depth_image(const krosc::msgs::Image& image);

  std::shared_ptr<Images> images_;
  QLabel* color_label_ = nullptr;
  QLabel* depth_label_ = nullptr;
  QLabel* color_info_ = nullptr;
  QLabel* depth_info_ = nullptr;
  QTimer* timer_ = nullptr;
  krosc::Subscription<krosc::msgs::Image> color_subscription_;
  krosc::Subscription<krosc::msgs::Image> depth_subscription_;
};

}  // namespace krosc_ws::viz
