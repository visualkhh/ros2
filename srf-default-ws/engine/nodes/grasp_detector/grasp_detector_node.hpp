#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "srf/msgs.hpp"
#include "srf/node.hpp"
#include "srf/tf/tf.hpp"
#include "srf/tf/tf_ros.hpp"

namespace srf_default_ws {

// 실기 D435 color/depth에서 집을 수 있는 색상 물체를 찾아 odom 좌표의
// sim_graspable Path로 발행한다. 시뮬레이터는 자체 world에서 같은 토픽을
// 발행하므로 이 노드는 실기(main.cpp)에서만 켠다.
//
// topic: camera/color/image_raw (jpeg), camera/depth/image_rect_raw (16UC1)
// out:   sim_graspable (Path, odom), grasp_info (String JSON)
class GraspDetectorNode final : public srf::Node {
 public:
  GraspDetectorNode(srf::Context& context, std::string color_topic,
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
  srf::msgs::Image color_;
  srf::msgs::Image depth_;
  bool color_seen_ = false;
  bool depth_seen_ = false;

  srf::Buffer tf_buffer_;
  std::unique_ptr<srf::TransformListener> tf_listener_;

  double last_log_ = 0.0;
  bool warned_tf_ = false;

  srf::Subscription<srf::msgs::Image> color_subscription_;
  srf::Subscription<srf::msgs::Image> depth_subscription_;
  srf::Timer timer_;
};

}  // namespace srf_default_ws
