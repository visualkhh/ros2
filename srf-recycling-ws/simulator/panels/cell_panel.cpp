#include "panels/cell_panel.hpp"

#include <QCheckBox>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

namespace srf_recycling_ws::sim {

namespace {
QString korean(Category c) {
  switch (c) {
    case Category::Can: return "캔";
    case Category::Pet: return "페트";
    case Category::Paper: return "종이";
    case Category::Unknown: return "불확실";
    case Category::Other: return "비대상";
  }
  return "";
}
}  // namespace

CellPanel::CellPanel(Post post, SharedCell& cell, QWidget* parent)
    : QWidget(parent), post_(std::move(post)), cell_(cell) {
  setObjectName("cell-panel");
  setMinimumWidth(280);
  setMaximumWidth(360);
  setStyleSheet(
      "#cell-panel { background: #11161f; color: #dbe4f0; }"
      "QLabel { color: #b8c4d6; }"
      "QCheckBox { color: #dbe4f0; }"
      "QPushButton { background: #1b2636; color: #dbe4f0; border: 1px solid #31425a;"
      " border-radius: 4px; padding: 5px 8px; }"
      "QPushButton:hover { background: #24344a; }");
  setAttribute(Qt::WA_StyledBackground, true);
  auto* layout = new QVBoxLayout(this);
  auto* title = new QLabel("작업 셀 (협동)");
  title->setStyleSheet("color: #dbe4f0; font-weight: bold; font-size: 14px;");
  layout->addWidget(title);

  auto* supply = new QPushButton("하역장 물품 공급 요청");
  connect(supply, &QPushButton::clicked, this, [this] { post_([](RecyclingWorld& w) { w.request_supply(); }); });
  layout->addWidget(supply);

  auto* grid = new QGridLayout;
  const std::array<Category, 3> kinds{Category::Can, Category::Pet, Category::Paper};
  for (int i = 0; i < 3; ++i) {
    const Category c = kinds[static_cast<std::size_t>(i)];
    bins_[static_cast<std::size_t>(i)] = new QLabel("-");
    auto* empty = new QPushButton(korean(c) + " 비우기");
    connect(empty, &QPushButton::clicked, this, [this, c] { post_([c](RecyclingWorld& w) { w.request_empty(c); }); });
    grid->addWidget(bins_[static_cast<std::size_t>(i)], i, 0);
    grid->addWidget(empty, i, 1);
  }
  layout->addLayout(grid);

  auto_supply_ = new QCheckBox("매트가 비면 자동 공급");
  auto_empty_ = new QCheckBox("바구니가 차면 자동 비우기");
  connect(auto_supply_, &QCheckBox::toggled, this, [this](bool on) {
    if (!updating_) post_([on](RecyclingWorld& w) { w.set_auto(on, w.scene().auto_empty); });
  });
  connect(auto_empty_, &QCheckBox::toggled, this, [this](bool on) {
    if (!updating_) post_([on](RecyclingWorld& w) { w.set_auto(w.scene().auto_supply, on); });
  });
  layout->addWidget(auto_supply_);
  layout->addWidget(auto_empty_);
  state_ = new QLabel("-");
  state_->setWordWrap(true);
  totals_ = new QLabel("-");
  totals_->setWordWrap(true);
  layout->addWidget(state_);
  layout->addWidget(totals_);

  auto* timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, [this] { refresh(); });
  timer->start(300);
}

void CellPanel::refresh() {
  CellSnapshot s;
  {
    std::lock_guard<std::mutex> lock(cell_.mutex);
    s = cell_.snapshot;
  }
  for (std::size_t i = 0; i < bins_.size() && i < s.bins.size(); ++i) {
    const auto& b = s.bins[i];
    const QString color = b.full ? "#ff5a4f" : b.count >= b.capacity - 1 ? "#ffad33" : "#5fd37a";
    bins_[i]->setText(QString("<span style='color:%1'>●</span> %2 %3/%4%5")
                          .arg(color, korean(b.category))
                          .arg(b.count)
                          .arg(b.capacity)
                          .arg(b.requested ? "  (호출됨)" : ""));
  }
  updating_ = true;
  auto_supply_->setChecked(s.auto_supply);
  auto_empty_->setChecked(s.auto_empty);
  updating_ = false;
  state_->setText(QString("셀 담당 작업자: %1  · 대기 작업 %2").arg(QString::fromStdString(s.state)).arg(s.queue));
  QString collected;
  for (const auto& [c, n] : s.collected) {
    collected += QString(" %1 %2").arg(korean(c)).arg(n);
  }
  totals_->setText(QString("공급 %1회 · 수거:%2").arg(s.supplies).arg(collected.isEmpty() ? " 없음" : collected));
}

}  // namespace srf_recycling_ws::sim
