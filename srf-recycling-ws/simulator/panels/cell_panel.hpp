#pragma once

// 협동 작업 셀 패널: 하역장 공급 요청, 바구니 비우기 호출, 자동 공급·비우기, 바구니 채움·셀 담당 작업자 상태.
// 월드는 시뮬레이션 스레드만 만진다 → 버튼은 post로 일을 넘기고, 상태는 시뮬레이션 스레드가 만든 복사본을 읽는다.
#include <QWidget>

#include <array>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "world/recycling_world.hpp"

class QCheckBox;
class QLabel;

namespace srf_recycling_ws::sim {

struct CellSnapshot {
  std::vector<RecyclingWorld::BinStatus> bins;
  std::string state;
  int queue = 0;
  int supplies = 0;
  std::map<Category, int> collected;
  bool auto_supply = true;
  bool auto_empty = true;
};

// 시뮬레이션 스레드가 채우고 패널이 읽는다.
struct SharedCell {
  std::mutex mutex;
  CellSnapshot snapshot;
};

class CellPanel final : public QWidget {
 public:
  using Post = std::function<void(std::function<void(RecyclingWorld&)>)>;
  CellPanel(Post post, SharedCell& cell, QWidget* parent = nullptr);

 private:
  void refresh();

  Post post_;
  SharedCell& cell_;
  std::array<QLabel*, 3> bins_{};
  QLabel* state_ = nullptr;
  QLabel* totals_ = nullptr;
  QCheckBox* auto_supply_ = nullptr;
  QCheckBox* auto_empty_ = nullptr;
  bool updating_ = false;
};

}  // namespace srf_recycling_ws::sim
