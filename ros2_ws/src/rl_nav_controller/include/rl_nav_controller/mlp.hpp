// 작은 MLP 추론 — isaac/training/rl_maze/export_mlp.py가 만든 policy.mlp (ONNX에서 뽑은 가중치). 외부 라이브러리 없음.
// 파일 (little-endian): "RLMLP1\0\0" · uint32 층 수 · uint32 활성(1 = ELU, 마지막 층 빼고) · 층마다 uint32 out · in · float W[out×in] · b[out]
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace rl_nav_controller
{

class Mlp
{
public:
  struct Layer
  {
    uint32_t out = 0, in = 0;
    std::vector<float> w, b;   // w: out×in (행 = 출력)
  };

  void load(const std::string & path)
  {
    std::ifstream f(path, std::ios::binary);
    if (!f) {throw std::runtime_error("policy.mlp 열기 실패: " + path);}
    char magic[8];
    uint32_t n = 0, act = 0;
    f.read(magic, 8);
    f.read(reinterpret_cast<char *>(&n), 4);
    f.read(reinterpret_cast<char *>(&act), 4);
    if (!f || std::memcmp(magic, "RLMLP1\0\0", 8) != 0 || act != 1 || n == 0 || n > 16) {
      throw std::runtime_error("policy.mlp 형식이 아님 (export_mlp.py로 만든 파일인지): " + path);
    }
    std::vector<Layer> layers(n);
    for (auto & l : layers) {
      f.read(reinterpret_cast<char *>(&l.out), 4);
      f.read(reinterpret_cast<char *>(&l.in), 4);
      if (!f || l.out == 0 || l.in == 0 || l.out > 4096 || l.in > 4096) {
        throw std::runtime_error("policy.mlp 층 크기 이상: " + path);
      }
      l.w.resize(size_t(l.out) * l.in);
      l.b.resize(l.out);
      f.read(reinterpret_cast<char *>(l.w.data()), std::streamsize(l.w.size() * 4));
      f.read(reinterpret_cast<char *>(l.b.data()), std::streamsize(l.b.size() * 4));
    }
    if (!f) {throw std::runtime_error("policy.mlp가 잘림: " + path);}
    for (size_t i = 1; i < n; ++i) {
      if (layers[i].in != layers[i - 1].out) {throw std::runtime_error("policy.mlp 층 연결 불일치: " + path);}
    }
    layers_ = std::move(layers);
  }

  bool loaded() const {return !layers_.empty();}
  uint32_t inputs() const {return loaded() ? layers_.front().in : 0;}
  uint32_t outputs() const {return loaded() ? layers_.back().out : 0;}

  std::vector<float> forward(const std::vector<float> & x) const
  {
    if (x.size() != inputs()) {throw std::runtime_error("MLP 입력 크기 불일치");}
    std::vector<float> a = x, z;
    for (size_t li = 0; li < layers_.size(); ++li) {
      const auto & l = layers_[li];
      z.assign(l.out, 0.0f);
      for (uint32_t o = 0; o < l.out; ++o) {
        float s = l.b[o];
        const float * row = &l.w[size_t(o) * l.in];
        for (uint32_t i = 0; i < l.in; ++i) {s += row[i] * a[i];}
        z[o] = (li + 1 < layers_.size() && s < 0.0f) ? std::expm1(s) : s;   // ELU (마지막 층 빼고)
      }
      a.swap(z);
    }
    return a;
  }

private:
  std::vector<Layer> layers_;
};

}  // namespace rl_nav_controller
