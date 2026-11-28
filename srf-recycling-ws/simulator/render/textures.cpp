#include "render/textures.hpp"

#include <QFont>
#include <QGuiApplication>
#include <QImage>
#include <QLinearGradient>
#include <QPainter>
#include <QPainterPath>
#include <QRadialGradient>

#include <cmath>
#include <cstring>
#include <functional>
#include <map>
#include <mutex>
#include <random>
#include <sstream>
#include <vector>

namespace srf_recycling_ws::sim::textures {

namespace {

// ---- 노이즈 ----

float hash2(int x, int y, std::uint32_t seed) {
  std::uint32_t h = static_cast<std::uint32_t>(x) * 374761393U + static_cast<std::uint32_t>(y) * 668265263U + seed * 2246822519U;
  h = (h ^ (h >> 13)) * 1274126177U;
  return static_cast<float>((h ^ (h >> 16)) & 0xFFFFFFU) / 16777215.0F;
}

// 반복되는 값 노이즈 (period 칸마다 이음매 없이).
float value_noise(float x, float y, int period, std::uint32_t seed) {
  const int x0 = static_cast<int>(std::floor(x)), y0 = static_cast<int>(std::floor(y));
  const float tx = x - x0, ty = y - y0;
  const auto s = [](float t) { return t * t * (3 - 2 * t); };
  const auto h = [&](int ix, int iy) {
    return hash2(((ix % period) + period) % period, ((iy % period) + period) % period, seed);
  };
  const float a = h(x0, y0), b = h(x0 + 1, y0), c = h(x0, y0 + 1), d = h(x0 + 1, y0 + 1);
  return (a + (b - a) * s(tx)) + ((c + (d - c) * s(tx)) - (a + (b - a) * s(tx))) * s(ty);
}

// 프랙탈 노이즈 0..1 (u,v ∈ [0,1) 에서 이음매 없음)
float fbm(float u, float v, int base, int octaves, std::uint32_t seed) {
  float sum = 0, amp = 0.5F, norm = 0;
  int period = base;
  for (int i = 0; i < octaves; ++i) {
    sum += amp * value_noise(u * period, v * period, period, seed + static_cast<std::uint32_t>(i) * 101U);
    norm += amp;
    amp *= 0.5F;
    period *= 2;
  }
  return sum / norm;
}

QColor shade(QColor c, float k) {
  return QColor::fromRgbF(std::clamp(c.redF() * k, 0.0F, 1.0F), std::clamp(c.greenF() * k, 0.0F, 1.0F),
                          std::clamp(c.blueF() * k, 0.0F, 1.0F), c.alphaF());
}

// 바탕색에 노이즈 얼룩을 입힌다.
QImage noisy(int size, QColor base, float strength, int scale, std::uint32_t seed, float speckle = 0.0F) {
  QImage image(size, size, QImage::Format_RGBA8888);
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> unit(0, 1);
  for (int y = 0; y < size; ++y) {
    for (int x = 0; x < size; ++x) {
      const float u = static_cast<float>(x) / size, v = static_cast<float>(y) / size;
      float k = 1.0F + (fbm(u, v, scale, 5, seed) - 0.5F) * 2 * strength;
      if (speckle > 0 && unit(rng) < speckle) {
        k *= 0.75F + 0.5F * unit(rng);
      }
      image.setPixelColor(x, y, shade(base, k));
    }
  }
  return image;
}

bool can_draw_text() { return QGuiApplication::instance() != nullptr; }

QFont font(int pixel_size, bool bold = true) {
  QFont f;
  f.setFamilies({"Apple SD Gothic Neo", "Noto Sans CJK KR", "NanumGothic", "Malgun Gothic", "Helvetica", "Arial"});
  f.setPixelSize(pixel_size);
  f.setBold(bold);
  return f;
}

void text(QPainter& p, const QRectF& rect, const QString& s, QColor color, int pixel_size, bool bold = true,
          int flags = Qt::AlignCenter) {
  if (!can_draw_text()) {
    return;
  }
  p.setFont(font(pixel_size, bold));
  p.setPen(color);
  p.drawText(rect, flags, s);
}

std::vector<std::string> split(const std::string& s, char sep) {
  std::vector<std::string> out;
  std::stringstream stream(s);
  for (std::string part; std::getline(stream, part, sep);) {
    out.push_back(part);
  }
  return out;
}

std::uint32_t seed_of(const std::string& name) {
  std::uint32_t h = 2166136261U;
  for (const char c : name) {
    h = (h ^ static_cast<std::uint8_t>(c)) * 16777619U;
  }
  return h;
}

// ---- 재질 ----

QImage concrete(std::uint32_t seed) {
  QImage img = noisy(512, QColor(150, 148, 142), 0.16F, 4, seed, 0.03F);
  QPainter p(&img);
  p.setRenderHint(QPainter::Antialiasing);
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(0, 512);
  // 기름 얼룩·타이어 자국
  for (int i = 0; i < 6; ++i) {
    QRadialGradient g(u(rng), u(rng), 30 + u(rng) / 6);
    g.setColorAt(0, QColor(60, 58, 55, 70));
    g.setColorAt(1, QColor(60, 58, 55, 0));
    p.setBrush(g);
    p.setPen(Qt::NoPen);
    p.drawRect(img.rect());
  }
  p.setPen(QPen(QColor(90, 88, 84, 120), 1.2));
  for (int i = 0; i < 3; ++i) {  // 잔금
    QPainterPath path;
    double x = u(rng), y = u(rng);
    path.moveTo(x, y);
    for (int k = 0; k < 8; ++k) {
      x += u(rng) / 16 - 16;
      y += u(rng) / 16 - 16;
      path.lineTo(x, y);
    }
    p.drawPath(path);
  }
  // 시공 줄눈 (1장 = 2 m 가정 → 가장자리 홈)
  p.setPen(QPen(QColor(80, 78, 74), 3));
  p.drawLine(0, 0, 512, 0);
  p.drawLine(0, 0, 0, 512);
  return img;
}

QImage epoxy(QColor base, std::uint32_t seed) {
  QImage img = noisy(512, base, 0.06F, 3, seed, 0.01F);
  QPainter p(&img);
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(0, 512);
  for (int i = 0; i < 40; ++i) {  // 긁힌 자국
    p.setPen(QPen(QColor(255, 255, 255, 25), 1));
    const double x = u(rng), y = u(rng);
    p.drawLine(QPointF(x, y), QPointF(x + u(rng) / 8 - 32, y + u(rng) / 20 - 12));
  }
  return img;
}

QImage wall_panel(std::uint32_t seed) {
  QImage img = noisy(256, QColor(196, 199, 201), 0.04F, 2, seed);
  QPainter p(&img);
  for (int x = 0; x < 256; x += 32) {  // 골판 샌드위치 패널 (세로 골)
    QLinearGradient g(x, 0, x + 32, 0);
    g.setColorAt(0.0, QColor(0, 0, 0, 30));
    g.setColorAt(0.15, QColor(255, 255, 255, 25));
    g.setColorAt(0.5, QColor(0, 0, 0, 0));
    g.setColorAt(0.9, QColor(0, 0, 0, 35));
    p.fillRect(x, 0, 32, 256, g);
  }
  return img;
}

QImage brushed(QColor base, std::uint32_t seed) {
  QImage img(256, 256, QImage::Format_RGBA8888);
  std::mt19937 rng(seed);
  std::normal_distribution<float> n(0, 0.05F);
  for (int y = 0; y < 256; ++y) {
    const float row = 1.0F + n(rng);
    for (int x = 0; x < 256; ++x) {
      img.setPixelColor(x, y, shade(base, row + n(rng) * 0.3F));
    }
  }
  return img;
}

QImage stripes(QColor a, QColor b, int count, bool diagonal) {
  QImage img(256, 256, QImage::Format_RGBA8888);
  for (int y = 0; y < 256; ++y) {
    for (int x = 0; x < 256; ++x) {
      const int t = diagonal ? (x + y) : x;
      img.setPixelColor(x, y, ((t * count / 256) % 2 == 0) ? a : b);
    }
  }
  return img;
}

QImage cardboard(std::uint32_t seed, bool printed) {
  QImage img = noisy(512, QColor(176, 136, 88), 0.10F, 6, seed, 0.02F);
  QPainter p(&img);
  p.setRenderHint(QPainter::Antialiasing);
  // 테이프
  p.fillRect(QRectF(0, 220, 512, 72), QColor(196, 160, 110, 200));
  p.setPen(QPen(QColor(150, 115, 70), 2));
  p.drawLine(0, 220, 512, 220);
  p.drawLine(0, 292, 512, 292);
  if (printed) {
    p.setPen(QPen(QColor(40, 30, 25, 200), 6));
    for (int i = 0; i < 2; ++i) {  // ↑↑ 화살표
      const double x = 80 + i * 60;
      p.drawLine(QPointF(x, 140), QPointF(x, 60));
      p.drawLine(QPointF(x, 60), QPointF(x - 18, 84));
      p.drawLine(QPointF(x, 60), QPointF(x + 18, 84));
    }
    text(p, QRectF(220, 50, 280, 100), "FRAGILE\n취급주의", QColor(40, 30, 25, 210), 40);
    text(p, QRectF(40, 360, 440, 120), "RECYCLE ♻", QColor(40, 30, 25, 200), 52);
  }
  return img;
}

QImage pallet_wood(std::uint32_t seed) {
  QImage img(256, 256, QImage::Format_RGBA8888);
  for (int y = 0; y < 256; ++y) {
    for (int x = 0; x < 256; ++x) {
      const float u = x / 256.0F, v = y / 256.0F;
      const float grain = std::sin((u * 18 + fbm(u, v, 3, 4, seed) * 6) * 3.14159F) * 0.5F + 0.5F;
      img.setPixelColor(x, y, shade(QColor(186, 150, 100), 0.82F + grain * 0.25F));
    }
  }
  return img;
}

QImage rubber_belt(std::uint32_t seed) {
  QImage img = noisy(256, QColor(42, 43, 45), 0.12F, 8, seed, 0.05F);
  QPainter p(&img);
  p.setPen(QPen(QColor(25, 25, 27), 3));
  for (int x = 0; x < 256; x += 16) {  // 벨트 결 (진행 방향 u에 수직 → 흘러가는 게 보인다)
    p.drawLine(x, 0, x, 256);
  }
  return img;
}

QImage rubber_mat(std::uint32_t seed) {
  QImage img = noisy(256, QColor(58, 60, 62), 0.08F, 6, seed);
  QPainter p(&img);
  p.setRenderHint(QPainter::Antialiasing);
  p.setPen(Qt::NoPen);
  for (int y = 8; y < 256; y += 16) {  // 미끄럼 방지 돌기
    for (int x = 8; x < 256; x += 16) {
      p.setBrush(QColor(80, 82, 85));
      p.drawEllipse(QPointF(x, y), 4, 4);
    }
  }
  return img;
}

QImage walkway(std::uint32_t seed) {
  QImage img = epoxy(QColor(64, 132, 86), seed);
  QPainter p(&img);
  p.setRenderHint(QPainter::Antialiasing);
  p.setBrush(QColor(235, 235, 230, 220));
  p.setPen(Qt::NoPen);
  QPainterPath arrow;  // 보행 방향 화살표 (오른쪽)
  arrow.moveTo(150, 236);
  arrow.lineTo(300, 236);
  arrow.lineTo(300, 200);
  arrow.lineTo(380, 256);
  arrow.lineTo(300, 312);
  arrow.lineTo(300, 276);
  arrow.lineTo(150, 276);
  arrow.closeSubpath();
  p.drawPath(arrow);
  text(p, QRectF(0, 360, 512, 100), "보행자 통로  WALKWAY", QColor(235, 235, 230, 220), 46);
  return img;
}

QImage jumbo_bag(std::uint32_t seed) {
  QImage img(256, 256, QImage::Format_RGBA8888);
  for (int y = 0; y < 256; ++y) {  // 짠 폴리 마대 (격자 결)
    for (int x = 0; x < 256; ++x) {
      const float weave = ((x / 3 + y / 3) % 2 == 0) ? 1.0F : 0.92F;
      const float n = fbm(x / 256.0F, y / 256.0F, 4, 4, seed);
      img.setPixelColor(x, y, shade(QColor(222, 220, 210), weave * (0.88F + n * 0.18F)));
    }
  }
  QPainter p(&img);
  p.fillRect(QRectF(0, 0, 18, 256), QColor(60, 90, 160));   // 멜빵 띠
  p.fillRect(QRectF(238, 0, 18, 256), QColor(60, 90, 160));
  text(p, QRectF(20, 60, 216, 120), "혼합\n재활용품", QColor(40, 60, 120), 30);
  return img;
}

QImage rollup_door(std::uint32_t seed) {
  QImage img = noisy(256, QColor(150, 154, 158), 0.05F, 2, seed);
  QPainter p(&img);
  for (int y = 0; y < 256; y += 12) {
    p.fillRect(0, y, 256, 2, QColor(100, 104, 108));
    p.fillRect(0, y + 2, 256, 1, QColor(200, 204, 208));
  }
  return img;
}

QImage bale(QColor a, QColor b, QColor c, std::uint32_t seed) {
  QImage img(256, 256, QImage::Format_RGBA8888);
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> pick(0, 2);
  const QColor colors[3] = {a, b, c};
  // 압축된 조각들 (가로로 길쭉한 얼룩)
  for (int y = 0; y < 256; y += 4) {
    for (int x = 0; x < 256;) {
      const int w = 6 + pick(rng) * 9;
      const QColor col = shade(colors[pick(rng)], 0.8F + 0.4F * hash2(x, y, seed));
      for (int dy = 0; dy < 4; ++dy) {
        for (int dx = 0; dx < w && x + dx < 256; ++dx) {
          img.setPixelColor(x + dx, y + dy, col);
        }
      }
      x += w;
    }
  }
  QPainter p(&img);
  for (int x : {60, 190}) {  // 철끈
    p.fillRect(x, 0, 8, 256, QColor(70, 70, 75));
  }
  return img;
}

QImage hi_vis_vest(std::uint32_t seed) {
  QImage img = noisy(128, QColor(240, 140, 30), 0.05F, 4, seed);  // 형광 주황
  QPainter p(&img);
  p.fillRect(0, 40, 128, 12, QColor(220, 220, 215));  // 반사띠
  p.fillRect(0, 80, 128, 12, QColor(220, 220, 215));
  return img;
}

QImage forklift_body(std::uint32_t seed) {
  QImage img = noisy(256, QColor(232, 180, 30), 0.06F, 3, seed, 0.01F);
  QPainter p(&img);
  p.fillRect(QRectF(0, 200, 256, 56), QColor(30, 30, 32));
  for (int x = -40; x < 256; x += 40) {  // 위험 사선
    QPainterPath path;
    path.moveTo(x, 256);
    path.lineTo(x + 20, 256);
    path.lineTo(x + 40, 200);
    path.lineTo(x + 20, 200);
    path.closeSubpath();
    p.fillPath(path, QColor(232, 180, 30));
  }
  text(p, QRectF(0, 40, 256, 100), "FL-02", QColor(30, 30, 32), 64);
  return img;
}

QImage crate(QColor base, const QString& label, std::uint32_t seed) {
  QImage img = noisy(256, base, 0.05F, 3, seed);
  QPainter p(&img);
  p.setRenderHint(QPainter::Antialiasing);
  // 플라스틱 바구니 손잡이 구멍·리브
  p.setBrush(shade(base, 0.55F));
  p.setPen(Qt::NoPen);
  p.drawRoundedRect(QRectF(88, 30, 80, 26), 12, 12);
  for (int x = 20; x < 256; x += 36) {
    p.fillRect(QRectF(x, 70, 6, 170), shade(base, 0.8F));
  }
  p.setBrush(QColor(255, 255, 255, 235));
  p.drawRoundedRect(QRectF(34, 110, 188, 96), 10, 10);
  text(p, QRectF(34, 110, 188, 96), label, shade(base, 0.6F), 40);
  return img;
}

QImage wheelie(QColor base, const QString& label, std::uint32_t seed) {
  QImage img = noisy(256, base, 0.05F, 3, seed, 0.01F);
  QPainter p(&img);
  p.setRenderHint(QPainter::Antialiasing);
  p.setBrush(QColor(255, 255, 255, 230));
  p.setPen(Qt::NoPen);
  p.drawRoundedRect(QRectF(40, 70, 176, 110), 10, 10);
  text(p, QRectF(40, 70, 176, 110), label, shade(base, 0.6F), 36);
  return img;
}

QImage cage_mesh() {
  QImage img(128, 128, QImage::Format_RGBA8888);
  img.fill(Qt::transparent);
  QPainter p(&img);
  p.setPen(QPen(QColor(150, 155, 160), 4));
  for (int i = 0; i <= 128; i += 16) {
    p.drawLine(i, 0, i, 128);
    p.drawLine(0, i, 128, i);
  }
  return img;
}

QImage glass() {
  QImage img(64, 64, QImage::Format_RGBA8888);
  img.fill(QColor(170, 200, 210, 80));
  QPainter p(&img);
  p.setPen(QPen(QColor(255, 255, 255, 120), 3));
  p.drawLine(8, 56, 30, 8);  // 반사 줄
  return img;
}

QImage shadow_blob() {
  QImage img(128, 128, QImage::Format_RGBA8888);
  img.fill(Qt::transparent);
  QPainter p(&img);
  QRadialGradient g(64, 64, 64);
  g.setColorAt(0.0, QColor(0, 0, 0, 170));
  g.setColorAt(0.55, QColor(0, 0, 0, 90));
  g.setColorAt(1.0, QColor(0, 0, 0, 0));
  p.fillRect(img.rect(), g);
  return img;
}

QImage newspaper(std::uint32_t seed) {
  QImage img = noisy(256, QColor(214, 210, 198), 0.04F, 4, seed);
  QPainter p(&img);
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> len(40, 110);
  text(p, QRectF(6, 4, 244, 40), "오늘신문 DAILY", QColor(30, 30, 30), 26);
  for (int col = 0; col < 2; ++col) {  // 기사 줄
    for (int y = 56; y < 250; y += 7) {
      p.fillRect(8 + col * 124, y, len(rng), 3, QColor(70, 70, 70));
    }
  }
  return img;
}

QImage vinyl(std::uint32_t seed) {
  QImage img(128, 128, QImage::Format_RGBA8888);
  for (int y = 0; y < 128; ++y) {
    for (int x = 0; x < 128; ++x) {
      const float n = fbm(x / 128.0F, y / 128.0F, 4, 4, seed);
      img.setPixelColor(x, y, QColor(235, 238, 240, static_cast<int>(120 + n * 100)));
    }
  }
  return img;
}

// ---- 물품 라벨 (원기둥에 감는다: 가로 = 둘레) ----

struct Brand {
  const char* key;
  QString name;
  QString sub;
  QColor bg;
  QColor fg;
};

const std::vector<Brand>& can_brands() {
  static const std::vector<Brand> b{
      {"cola", "콜라", "COLA", QColor(200, 20, 30), QColor(255, 255, 255)},
      {"cider", "사이다", "CIDER", QColor(30, 150, 70), QColor(255, 255, 255)},
      {"coffee", "커피", "COFFEE", QColor(70, 40, 25), QColor(235, 200, 140)},
      {"beer", "맥주", "LAGER", QColor(230, 190, 40), QColor(40, 30, 20)},
      {"energy", "에너지", "ENERGY", QColor(20, 20, 25), QColor(120, 230, 60)},
      {"tuna", "참치", "TUNA", QColor(30, 80, 160), QColor(255, 220, 60)},
  };
  return b;
}

const std::vector<Brand>& pet_brands() {
  static const std::vector<Brand> b{
      {"water", "생수", "WATER", QColor(40, 120, 210), QColor(255, 255, 255)},
      {"soda", "탄산수", "SPARKLING", QColor(210, 40, 50), QColor(255, 255, 255)},
      {"tea", "녹차", "GREEN TEA", QColor(60, 140, 60), QColor(255, 255, 230)},
      {"juice", "오렌지", "ORANGE", QColor(245, 140, 20), QColor(255, 255, 255)},
  };
  return b;
}

QImage can_label(const Brand& brand, std::uint32_t seed) {
  QImage img(512, 256, QImage::Format_RGBA8888);
  QPainter p(&img);
  p.setRenderHint(QPainter::Antialiasing);
  QLinearGradient g(0, 0, 0, 256);
  g.setColorAt(0, shade(brand.bg, 1.15F));
  g.setColorAt(1, shade(brand.bg, 0.85F));
  p.fillRect(img.rect(), g);
  // 물결 띠·로고 원 (반대편에도 하나)
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(brand.fg.red(), brand.fg.green(), brand.fg.blue(), 60));
  QPainterPath wave;
  wave.moveTo(0, 170);
  for (int x = 0; x <= 512; x += 16) {
    wave.lineTo(x, 170 + 14 * std::sin(x / 512.0 * 6.283 * 2 + seed % 7));
  }
  wave.lineTo(512, 256);
  wave.lineTo(0, 256);
  p.drawPath(wave);
  for (const double cx : {128.0, 384.0}) {
    text(p, QRectF(cx - 120, 40, 240, 90), brand.name, brand.fg, 64);
    text(p, QRectF(cx - 120, 120, 240, 50), brand.sub, brand.fg, 30);
  }
  text(p, QRectF(0, 210, 512, 40), "♻ 캔류  ALU  355ml", QColor(255, 255, 255, 200), 18, false);
  return img;
}

QImage pet_label(const Brand& brand) {
  QImage img(512, 128, QImage::Format_RGBA8888);
  QPainter p(&img);
  p.setRenderHint(QPainter::Antialiasing);
  p.fillRect(img.rect(), brand.bg);
  p.fillRect(QRectF(0, 0, 512, 10), QColor(255, 255, 255));
  p.fillRect(QRectF(0, 118, 512, 10), QColor(255, 255, 255));
  for (const double cx : {128.0, 384.0}) {
    text(p, QRectF(cx - 120, 14, 240, 64), brand.name, brand.fg, 48);
    text(p, QRectF(cx - 120, 74, 240, 40), brand.sub, brand.fg, 22);
  }
  text(p, QRectF(0, 96, 512, 22), "♻ PET  500ml", QColor(255, 255, 255, 220), 14, false);
  return img;
}

QImage milk_label() {
  QImage img(512, 256, QImage::Format_RGBA8888);  // 네 옆면을 가로로 이어 붙인 전개도
  img.fill(QColor(246, 246, 242));
  QPainter p(&img);
  p.setRenderHint(QPainter::Antialiasing);
  for (int side = 0; side < 4; ++side) {
    const double x = side * 128;
    p.fillRect(QRectF(x, 150, 128, 106), QColor(40, 100, 200));
    p.setBrush(QColor(255, 255, 255));
    p.setPen(Qt::NoPen);
    p.drawEllipse(QPointF(x + 64, 178), 30, 16);  // 구름 무늬
    text(p, QRectF(x, 30, 128, 70), side % 2 == 0 ? "우유" : "MILK", QColor(40, 100, 200), side % 2 == 0 ? 46 : 36);
    text(p, QRectF(x, 96, 128, 40), "1A 신선", QColor(80, 80, 80), 18, false);
  }
  text(p, QRectF(0, 226, 512, 28), "♻ 종이팩", QColor(255, 255, 255), 18, false);
  return img;
}

QImage cup_label() {
  QImage img(512, 256, QImage::Format_RGBA8888);
  img.fill(QColor(248, 248, 246));
  QPainter p(&img);
  p.setRenderHint(QPainter::Antialiasing);
  for (const double cx : {128.0, 384.0}) {
    p.setBrush(QColor(20, 110, 80));
    p.setPen(Qt::NoPen);
    p.drawEllipse(QPointF(cx, 110), 56, 56);
    text(p, QRectF(cx - 56, 54, 112, 112), "CAFE", QColor(255, 255, 255), 30);
  }
  p.fillRect(QRectF(0, 200, 512, 56), QColor(140, 95, 55));  // 슬리브
  text(p, QRectF(0, 200, 512, 56), "HOT  뜨거워요", QColor(250, 240, 225), 22);
  return img;
}

QImage tetra_label() {
  QImage img(512, 256, QImage::Format_RGBA8888);
  img.fill(QColor(250, 200, 40));
  QPainter p(&img);
  p.setRenderHint(QPainter::Antialiasing);
  for (int side = 0; side < 4; ++side) {
    const double x = side * 128;
    p.setBrush(QColor(245, 130, 20));
    p.setPen(Qt::NoPen);
    p.drawEllipse(QPointF(x + 64, 150), 40, 40);
    text(p, QRectF(x, 20, 128, 60), "주스", QColor(120, 50, 10), 40);
  }
  text(p, QRectF(0, 210, 512, 40), "멸균팩 (알루미늄 포함)", QColor(120, 50, 10), 18, false);
  return img;
}

QImage box_label(std::uint32_t seed) {
  QImage img = cardboard(seed, false);
  QPainter p(&img);
  text(p, QRectF(30, 40, 452, 140), "택배  PARCEL", QColor(40, 30, 25, 220), 64);
  p.fillRect(QRectF(300, 330, 180, 120), QColor(255, 255, 255, 230));  // 송장
  p.setPen(QColor(40, 40, 40));
  for (int y = 345; y < 440; y += 14) {
    p.drawLine(310, y, 460, y);
  }
  return img;
}

QImage sign(const QString& label, QColor bg, QColor fg) {
  QImage img(1024, 256, QImage::Format_RGBA8888);
  img.fill(bg);
  QPainter p(&img);
  p.setRenderHint(QPainter::Antialiasing);
  p.setPen(QPen(fg, 10));
  p.setBrush(Qt::NoBrush);
  p.drawRoundedRect(QRectF(14, 14, 996, 228), 20, 20);
  text(p, QRectF(30, 20, 964, 216), label, fg, label.size() > 14 ? 64 : 96);
  return img;
}

QImage make(const std::string& name) {
  const std::uint32_t seed = seed_of(name);
  const auto parts = split(name, '/');
  if (parts.size() >= 4 && parts[0] == "sign") {
    return sign(QString::fromStdString(parts[1]), QColor(QString::fromStdString(parts[2])),
                QColor(QString::fromStdString(parts[3])));
  }
  if (parts.size() >= 3 && parts[0] == "label" && parts[1] == "can") {
    for (const auto& b : can_brands()) {
      if (parts[2] == b.key) return can_label(b, seed);
    }
    return can_label(can_brands()[0], seed);
  }
  if (parts.size() >= 3 && parts[0] == "label" && parts[1] == "pet") {
    for (const auto& b : pet_brands()) {
      if (parts[2] == b.key) return pet_label(b);
    }
    return pet_label(pet_brands()[0]);
  }
  static const std::map<std::string, std::function<QImage(std::uint32_t)>> table{
      {"concrete", concrete},
      {"epoxy_gray", [](std::uint32_t s) { return epoxy(QColor(120, 126, 128), s); }},
      {"epoxy_green", [](std::uint32_t s) { return epoxy(QColor(92, 120, 104), s); }},
      {"wall_panel", wall_panel},
      {"steel_brushed", [](std::uint32_t s) { return brushed(QColor(150, 155, 160), s); }},
      {"aluminum", [](std::uint32_t s) { return brushed(QColor(205, 208, 212), s); }},
      {"steel_yellow", [](std::uint32_t s) { return noisy(128, QColor(225, 180, 35), 0.05F, 3, s, 0.01F); }},
      {"steel_blue", [](std::uint32_t s) { return noisy(128, QColor(40, 80, 150), 0.05F, 3, s, 0.01F); }},
      {"rubber_belt", rubber_belt},
      {"rubber_mat", rubber_mat},
      {"pallet_wood", pallet_wood},
      {"cardboard", [](std::uint32_t s) { return cardboard(s, true); }},
      {"kraft", [](std::uint32_t s) { return cardboard(s, false); }},
      {"hazard_stripe", [](std::uint32_t) { return stripes(QColor(235, 190, 20), QColor(25, 25, 25), 8, true); }},
      {"tape_yellow", [](std::uint32_t s) { return noisy(64, QColor(235, 195, 30), 0.05F, 2, s); }},
      {"walkway", walkway},
      {"jumbo_bag", jumbo_bag},
      {"rollup_door", rollup_door},
      {"bale_cardboard", [](std::uint32_t s) { return bale(QColor(170, 130, 85), QColor(150, 115, 75), QColor(200, 190, 170), s); }},
      {"bale_pet", [](std::uint32_t s) { return bale(QColor(180, 215, 230), QColor(60, 140, 200), QColor(220, 230, 235), s); }},
      {"bale_can", [](std::uint32_t s) { return bale(QColor(190, 192, 196), QColor(200, 40, 40), QColor(140, 145, 150), s); }},
      {"hi_vis_vest", hi_vis_vest},
      {"denim", [](std::uint32_t s) { return noisy(64, QColor(55, 75, 115), 0.10F, 8, s); }},
      {"shirt_blue", [](std::uint32_t s) { return noisy(64, QColor(60, 90, 140), 0.06F, 4, s); }},
      {"shirt_gray", [](std::uint32_t s) { return noisy(64, QColor(110, 112, 116), 0.06F, 4, s); }},
      {"forklift_body", forklift_body},
      {"crate_can", [](std::uint32_t s) { return crate(QColor(205, 45, 40), "캔류\nCAN", s); }},
      {"crate_pet", [](std::uint32_t s) { return crate(QColor(40, 100, 200), "페트병\nPET", s); }},
      {"crate_paper", [](std::uint32_t s) { return crate(QColor(45, 150, 70), "종이류\nPAPER", s); }},
      {"wheelie_blue", [](std::uint32_t s) { return wheelie(QColor(40, 90, 170), "플라스틱", s); }},
      {"wheelie_green", [](std::uint32_t s) { return wheelie(QColor(40, 130, 60), "유리병", s); }},
      {"wheelie_yellow", [](std::uint32_t s) { return wheelie(QColor(220, 175, 30), "캔·고철", s); }},
      {"wheelie_gray", [](std::uint32_t s) { return wheelie(QColor(110, 115, 120), "일반쓰레기", s); }},
      {"cage_mesh", [](std::uint32_t) { return cage_mesh(); }},
      {"glass", [](std::uint32_t) { return glass(); }},
      {"shadow_blob", [](std::uint32_t) { return shadow_blob(); }},
      {"newspaper", newspaper},
      {"styrofoam", [](std::uint32_t s) { return noisy(64, QColor(245, 245, 242), 0.03F, 8, s, 0.08F); }},
      {"vinyl", vinyl},
      {"label/milk", [](std::uint32_t) { return milk_label(); }},
      {"label/cup", [](std::uint32_t) { return cup_label(); }},
      {"label/tetra", [](std::uint32_t) { return tetra_label(); }},
      {"label/box", box_label},
  };
  const auto found = table.find(name);
  if (found != table.end()) {
    return found->second(seed);
  }
  QImage fallback(8, 8, QImage::Format_RGBA8888);  // 모르는 이름: 분홍 (눈에 띄게)
  fallback.fill(QColor(255, 0, 255));
  return fallback;
}

}  // namespace

std::shared_ptr<render::Texture> from_image(const QImage& image) {
  const QImage rgba = image.convertToFormat(QImage::Format_RGBA8888);
  auto texture = std::make_shared<render::Texture>();
  texture->width = rgba.width();
  texture->height = rgba.height();
  texture->rgba.resize(static_cast<std::size_t>(rgba.width()) * static_cast<std::size_t>(rgba.height()) * 4);
  for (int y = 0; y < rgba.height(); ++y) {
    std::memcpy(&texture->rgba[static_cast<std::size_t>(y) * static_cast<std::size_t>(rgba.width()) * 4],
                rgba.constScanLine(y), static_cast<std::size_t>(rgba.width()) * 4);
  }
  return texture;
}

std::shared_ptr<const render::Texture> get(const std::string& name) {
  static std::mutex mutex;
  static std::map<std::string, std::shared_ptr<const render::Texture>> cache;
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (const auto found = cache.find(name); found != cache.end()) {
      return found->second;
    }
  }
  auto texture = from_image(make(name));  // 잠금 밖에서 만든다 (느릴 수 있다)
  std::lock_guard<std::mutex> lock(mutex);
  return cache.emplace(name, std::move(texture)).first->second;
}

}  // namespace srf_recycling_ws::sim::textures
