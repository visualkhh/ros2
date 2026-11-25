#pragma once

#include <map>
#include <string>

#include "krosc/node.hpp"
#include "krosc/tf/tf.hpp"
#include "krosc/tf/tf_ros.hpp"
#include "krosc/time/timer.hpp"

namespace krosc::nodes {

// target_frame에서 본 source_frame의 변환을 rate(Hz)마다 로그로 찍는다 (tf2_echo).
class TfEcho final : public Node {
 public:
  explicit TfEcho(Context& context, std::string name = "tf_echo", std::map<std::string, ParamValue> parameters = {});

 private:
  void tick();

  Parameter<std::string> target_frame_;
  Parameter<std::string> source_frame_;
  Parameter<double> rate_;
  Buffer buffer_;
  TransformListener listener_;
  Timer timer_;
  std::string last_error_;
};

}  // namespace krosc::nodes
