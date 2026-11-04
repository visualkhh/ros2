// corrector 계산식 (학습 env와 같은 약속) — 노드와 테스트가 같이 쓴다.
#pragma once

#include <algorithm>
#include <cmath>
#include <utility>

namespace rl_correction
{

inline double wrap_to_pi(double a)
{
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  return (a < 0 ? a + 2.0 * M_PI : a) - M_PI;
}

// 정책 출력 [-1, 1] → 물리 보정량 [dV m/s, dW rad/s] (학습 env와 같은 clamp × action_scale)
inline std::pair<double, double> scale_action(float o0, float o1, double scale_lin, double scale_ang)
{
  return {std::clamp(double(o0), -1.0, 1.0) * scale_lin, std::clamp(double(o1), -1.0, 1.0) * scale_ang};
}

// 보정을 더하고 자른다. 0 명령은 0 그대로 (sim도 같은 규칙)
inline std::pair<double, double> correct(
  std::pair<double, double> cmd, std::pair<double, double> delta, double max_lin, double max_ang)
{
  if (cmd.first == 0.0 && cmd.second == 0.0) {return {0.0, 0.0};}
  return {std::clamp(cmd.first + delta.first, -max_lin, max_lin),
    std::clamp(cmd.second + delta.second, -max_ang, max_ang)};
}

}  // namespace rl_correction
