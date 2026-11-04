// test_correct.py를 옮긴 것 — 0 명령 · 더하고 자르기 · clamp 뒤 스케일
#include <cassert>
#include <cmath>
#include <cstdio>

#include "rl_correction/correct.hpp"

static bool near(double a, double b) {return std::fabs(a - b) < 1e-9;}

int main()
{
  using rl_correction::correct;
  using rl_correction::scale_action;
  auto z = correct({0.0, 0.0}, {0.01, 0.01}, 0.26, 1.82);
  assert(z.first == 0.0 && z.second == 0.0);
  auto a = correct({0.2, 0.0}, {0.01, -0.02}, 0.26, 1.82);
  assert(near(a.first, 0.21) && near(a.second, -0.02));
  auto c = correct({0.25, 1.8}, {0.1, 0.5}, 0.26, 1.82);
  assert(near(c.first, 0.26) && near(c.second, 1.82));
  auto s1 = scale_action(0.5f, -2.0f, 0.10, 0.60);
  assert(near(s1.first, 0.05) && near(s1.second, -0.60));
  auto s2 = scale_action(3.0f, 0.1f, 0.10, 0.60);
  assert(near(s2.first, 0.10) && std::fabs(s2.second - 0.06) < 1e-7);
  assert(near(rl_correction::wrap_to_pi(3.0 * M_PI), M_PI) || near(rl_correction::wrap_to_pi(3.0 * M_PI), -M_PI));
  std::printf("ok\n");
  return 0;
}
