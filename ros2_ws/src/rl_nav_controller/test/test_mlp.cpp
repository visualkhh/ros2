// policy.mlp 읽기 · 계산 확인 (외부 라이브러리 없음): 2→3→1, ELU — 손으로 계산한 값과 비교
#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>

#include "rl_nav_controller/mlp.hpp"

static void put(std::ofstream & f, uint32_t v) {f.write(reinterpret_cast<const char *>(&v), 4);}
static void put(std::ofstream & f, float v) {f.write(reinterpret_cast<const char *>(&v), 4);}

int main()
{
  const char * path = "/tmp/rl_nav_controller_test.mlp";
  {
    std::ofstream f(path, std::ios::binary);
    f.write("RLMLP1\0\0", 8);
    put(f, uint32_t(2));
    put(f, uint32_t(1));
    put(f, uint32_t(3)); put(f, uint32_t(2));               // 층 1: 3×2
    for (float v : {1.f, 0.f, 0.f, 1.f, -1.f, -1.f}) {put(f, v);}
    for (float v : {0.f, 0.f, 0.f}) {put(f, v);}
    put(f, uint32_t(1)); put(f, uint32_t(3));               // 층 2: 1×3
    for (float v : {1.f, 1.f, 1.f}) {put(f, v);}
    put(f, 0.5f);
  }
  rl_nav_controller::Mlp m;
  m.load(path);
  assert(m.inputs() == 2 && m.outputs() == 1);
  const auto y = m.forward({2.0f, -1.0f});   // h = [2, -1, -1] → ELU [2, e⁻¹−1, e⁻¹−1] → 합 + 0.5
  const float want = 2.0f + 2.0f * std::expm1(-1.0f) + 0.5f;
  assert(std::fabs(y[0] - want) < 1e-5f);
  bool threw = false;
  try {m.forward({1.0f});} catch (...) {threw = true;}
  assert(threw);
  std::printf("ok %f\n", y[0]);
  return 0;
}
