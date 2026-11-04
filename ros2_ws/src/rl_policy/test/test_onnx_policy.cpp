// tiny.onnx (y = x·W + b, 입력 3 · 출력 2 — test/data/make_tiny.py) 를 읽어 손으로 계산한 값과 비교
#include <cassert>
#include <cmath>
#include <cstdio>

#include "rl_policy/onnx_policy.hpp"

int main(int argc, char ** argv)
{
  assert(argc == 2);
  rl_policy::OnnxPolicy p;
  assert(!p.loaded());
  p.load(argv[1]);
  assert(p.loaded() && p.inputs() == 3 && p.outputs() == 2);
  const auto y = p.run({1.0f, 2.0f, 3.0f});   // W = [[1, 0], [0, 1], [1, -1]] · b = [0.5, -0.5]
  assert(std::fabs(y[0] - 4.5f) < 1e-5f && std::fabs(y[1] - (-1.5f)) < 1e-5f);
  bool threw = false;
  try {p.run({1.0f});} catch (...) {threw = true;}
  assert(threw);
  threw = false;
  try {p.load("/nonexistent.onnx");} catch (...) {threw = true;}
  assert(threw && p.loaded());   // 실패해도 이전 모델 유지
  std::printf("ok %f %f\n", y[0], y[1]);
  return 0;
}
