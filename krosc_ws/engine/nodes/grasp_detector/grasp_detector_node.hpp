#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "krosc/msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/tf/tf.hpp"
#include "krosc/tf/tf_ros.hpp"

namespace krosc_ws {

// 실기 D435 color/depth에서 집을 수 있는 색상 물체를 찾아 odom 좌표의
// sim_graspable Path로 발행한다. 시뮬레이터는 자체 world에서 같은 토픽을
// 발행하므로 이 노드는 실기(main.cpp)에서만 켠다.
//
// topic: camera/color/image_raw (jpeg), camera/depth/image_rect_raw (16UC1)
// out:   sim_graspable (Path, odom), grasp_info (String JSON)
class GraspDetectorNode final : public krosc::Node {
 public:
  GraspDetectorNode(krosc::Context& context, std::string color_topic,
                    std::string depth_topic, std::string objects_topic,
                    std::string info_topic);
  ~GraspDetectorNode() override;

  GraspDetectorNode(const GraspDetectorNode&) = delete;
  GraspDetectorNode& operator=(const GraspDetectorNode&) = delete;

 private:
  void detect();

  std::string color_topic_;
  std::string depth_topic_;
  std::string objects_topic_;
  std::string info_topic_;

  mutable std::mutex image_mutex_;
  krosc::msgs::Image color_;
  krosc::msgs::Image depth_;
  bool color_seen_ = false;
  bool depth_seen_ = false;

  krosc::Buffer tf_buffer_;
  std::unique_ptr<krosc::TransformListener> tf_listener_;

  double last_log_ = 0.0;
  bool warned_tf_ = false;

  krosc::Subscription<krosc::msgs::Image> color_subscription_;
  krosc::Subscription<krosc::msgs::Image> depth_subscription_;
  krosc::Timer timer_;
};

}  // namespace krosc_ws
