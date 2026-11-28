#include "render/scene_renderer.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace srf_recycling_ws::sim::render {

namespace {

constexpr double kPi = 3.14159265358979323846;

Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(const Vec3& a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
Vec3 normalized(const Vec3& v) {
  const double n = std::sqrt(dot(v, v));
  return n > 1e-12 ? v * (1.0 / n) : Vec3{0, 0, 1};
}
Vec3 to_d(const F3& f) { return {f.x, f.y, f.z}; }
F3 to_f(const Vec3& v) { return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)}; }

}  // namespace

// ---- Pose ----

Vec3 Pose::rotate(const Vec3& v) const {
  return {r[0][0] * v.x + r[0][1] * v.y + r[0][2] * v.z, r[1][0] * v.x + r[1][1] * v.y + r[1][2] * v.z,
          r[2][0] * v.x + r[2][1] * v.y + r[2][2] * v.z};
}

Vec3 Pose::apply(const Vec3& v) const { return rotate(v) + t; }

Pose Pose::operator*(const Pose& o) const {
  Pose out;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out.r[i][j] = r[i][0] * o.r[0][j] + r[i][1] * o.r[1][j] + r[i][2] * o.r[2][j];
    }
  }
  out.t = apply(o.t);
  return out;
}

Pose Pose::inverse() const {
  Pose out;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out.r[i][j] = r[j][i];
    }
  }
  out.t = out.rotate(t) * -1.0;
  return out;
}

Pose Pose::translation(double x, double y, double z) {
  Pose out;
  out.t = {x, y, z};
  return out;
}

Pose Pose::rpy(const std::array<double, 3>& xyz, const std::array<double, 3>& rpy) {
  const double cr = std::cos(rpy[0]), sr = std::sin(rpy[0]);
  const double cp = std::cos(rpy[1]), sp = std::sin(rpy[1]);
  const double cy = std::cos(rpy[2]), sy = std::sin(rpy[2]);
  Pose out;
  out.r = {{{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
            {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
            {-sp, cp * sr, cp * cr}}};
  out.t = {xyz[0], xyz[1], xyz[2]};
  return out;
}

Pose Pose::axis_angle(const std::array<double, 3>& axis, double angle) {
  const Vec3 a = normalized({axis[0], axis[1], axis[2]});
  const double c = std::cos(angle), s = std::sin(angle), k = 1.0 - c;
  Pose out;
  out.r = {{{c + a.x * a.x * k, a.x * a.y * k - a.z * s, a.x * a.z * k + a.y * s},
            {a.y * a.x * k + a.z * s, c + a.y * a.y * k, a.y * a.z * k - a.x * s},
            {a.z * a.x * k - a.y * s, a.z * a.y * k + a.x * s, c + a.z * a.z * k}}};
  return out;
}

Pose Pose::yaw(double angle) { return axis_angle({0, 0, 1}, angle); }

Pose Pose::scale_free_look(const Vec3& eye, const Vec3& target) {
  const Vec3 z = normalized(target - eye);
  Vec3 x = cross(z, {0, 0, 1});
  x = dot(x, x) < 1e-9 ? Vec3{1, 0, 0} : normalized(x);  // 오른쪽
  const Vec3 y = cross(z, x);                             // 아래
  Pose pose;
  pose.r = {{{x.x, y.x, z.x}, {x.y, y.y, z.y}, {x.z, y.z, z.z}}};
  pose.t = eye;
  return pose;
}

// ---- 텍스처 ----

std::array<float, 4> Texture::sample(float u, float v) const {
  if (width <= 0 || height <= 0) {
    return {1, 1, 1, 1};
  }
  const float fx = (u - std::floor(u)) * width - 0.5F;
  const float fy = (v - std::floor(v)) * height - 0.5F;
  const int x0 = static_cast<int>(std::floor(fx)), y0 = static_cast<int>(std::floor(fy));
  const float tx = fx - x0, ty = fy - y0;
  const auto at = [&](int x, int y) {
    x = ((x % width) + width) % width;
    y = ((y % height) + height) % height;
    return &rgba[(static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x)) * 4];
  };
  const std::uint8_t* a = at(x0, y0);
  const std::uint8_t* b = at(x0 + 1, y0);
  const std::uint8_t* c = at(x0, y0 + 1);
  const std::uint8_t* d = at(x0 + 1, y0 + 1);
  std::array<float, 4> out{};
  constexpr float k = 1.0F / 255.0F;
  for (int i = 0; i < 4; ++i) {
    out[static_cast<std::size_t>(i)] =
        ((a[i] * (1 - tx) + b[i] * tx) * (1 - ty) + (c[i] * (1 - tx) + d[i] * tx) * ty) * k;
  }
  return out;
}

// ---- 메시 ----

void Mesh::add(const std::array<F3, 3>& p, const std::array<F3, 3>& n, const std::array<UV, 3>& uv) {
  positions.push_back(p);
  normals.push_back(n);
  uvs.push_back(uv);
}

void Mesh::finish() {
  if (positions.empty()) {
    return;
  }
  Vec3 low{1e9, 1e9, 1e9};
  Vec3 high{-1e9, -1e9, -1e9};
  for (const auto& tri : positions) {
    for (const auto& v : tri) {
      low = {std::min(low.x, static_cast<double>(v.x)), std::min(low.y, static_cast<double>(v.y)),
             std::min(low.z, static_cast<double>(v.z))};
      high = {std::max(high.x, static_cast<double>(v.x)), std::max(high.y, static_cast<double>(v.y)),
              std::max(high.z, static_cast<double>(v.z))};
    }
  }
  center = (low + high) * 0.5;
  radius = std::sqrt(dot(high - center, high - center));
}

std::shared_ptr<Mesh> make_box(double sx, double sy, double sz) {
  auto mesh = std::make_shared<Mesh>();
  const float x = static_cast<float>(sx / 2), y = static_cast<float>(sy / 2), z = static_cast<float>(sz / 2);
  // 면: 법선, 네 꼭짓점(바깥에서 볼 때 왼쪽 아래부터 반시계), UV
  struct Face {
    F3 n;
    std::array<F3, 4> p;
  };
  const std::array<Face, 6> faces{{
      {{0, 0, 1}, {{{-x, -y, z}, {x, -y, z}, {x, y, z}, {-x, y, z}}}},
      {{0, 0, -1}, {{{-x, y, -z}, {x, y, -z}, {x, -y, -z}, {-x, -y, -z}}}},
      {{1, 0, 0}, {{{x, -y, -z}, {x, y, -z}, {x, y, z}, {x, -y, z}}}},
      {{-1, 0, 0}, {{{-x, y, -z}, {-x, -y, -z}, {-x, -y, z}, {-x, y, z}}}},
      {{0, 1, 0}, {{{x, y, -z}, {-x, y, -z}, {-x, y, z}, {x, y, z}}}},
      {{0, -1, 0}, {{{-x, -y, -z}, {x, -y, -z}, {x, -y, z}, {-x, -y, z}}}},
  }};
  const std::array<UV, 4> uv{{{0, 1}, {1, 1}, {1, 0}, {0, 0}}};
  for (const auto& f : faces) {
    mesh->add({f.p[0], f.p[1], f.p[2]}, {f.n, f.n, f.n}, {uv[0], uv[1], uv[2]});
    mesh->add({f.p[0], f.p[2], f.p[3]}, {f.n, f.n, f.n}, {uv[0], uv[2], uv[3]});
  }
  mesh->finish();
  return mesh;
}

namespace {

// 옆면 반지름 함수(높이 비율 t∈[0,1], 각도 a)로 회전체를 만든다.
template <class RadiusFn>
std::shared_ptr<Mesh> make_revolved(RadiusFn radius_at, double length, int segments, int rings, bool bottom_cap,
                                    bool top_cap) {
  auto mesh = std::make_shared<Mesh>();
  const double z0 = -length / 2;
  const auto point = [&](int ring, int seg) {
    const double t = static_cast<double>(ring) / rings;
    const double a = 2 * kPi * seg / segments;
    const double r = radius_at(t, a);
    return Vec3{r * std::cos(a), r * std::sin(a), z0 + t * length};
  };
  std::vector<std::vector<Vec3>> grid(static_cast<std::size_t>(rings + 1),
                                      std::vector<Vec3>(static_cast<std::size_t>(segments + 1)));
  for (int i = 0; i <= rings; ++i) {
    for (int j = 0; j <= segments; ++j) {
      grid[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = point(i, j % segments);
    }
  }
  // 정점 법선: 이웃 면 법선의 평균 (부드러운 옆면)
  const auto normal_at = [&](int i, int j) {
    const auto& g = grid;
    const auto up = g[static_cast<std::size_t>(std::min(i + 1, rings))][static_cast<std::size_t>(j)];
    const auto down = g[static_cast<std::size_t>(std::max(i - 1, 0))][static_cast<std::size_t>(j)];
    const int jl = (j + segments - 1) % segments, jr = (j + 1) % segments;
    const auto left = g[static_cast<std::size_t>(i)][static_cast<std::size_t>(jl)];
    const auto right = g[static_cast<std::size_t>(i)][static_cast<std::size_t>(jr)];
    Vec3 n = cross(right - left, up - down);
    return normalized(n);
  };
  for (int i = 0; i < rings; ++i) {
    for (int j = 0; j < segments; ++j) {
      const auto& a = grid[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
      const auto& b = grid[static_cast<std::size_t>(i)][static_cast<std::size_t>(j + 1)];
      const auto& c = grid[static_cast<std::size_t>(i + 1)][static_cast<std::size_t>(j + 1)];
      const auto& d = grid[static_cast<std::size_t>(i + 1)][static_cast<std::size_t>(j)];
      const F3 na = to_f(normal_at(i, j)), nb = to_f(normal_at(i, (j + 1) % segments));
      const F3 nc = to_f(normal_at(i + 1, (j + 1) % segments)), nd = to_f(normal_at(i + 1, j));
      const float u0 = static_cast<float>(j) / segments, u1 = static_cast<float>(j + 1) / segments;
      const float v0 = 1.0F - static_cast<float>(i) / rings, v1 = 1.0F - static_cast<float>(i + 1) / rings;
      mesh->add({to_f(a), to_f(b), to_f(c)}, {na, nb, nc}, {UV{u0, v0}, UV{u1, v0}, UV{u1, v1}});
      mesh->add({to_f(a), to_f(c), to_f(d)}, {na, nc, nd}, {UV{u0, v0}, UV{u1, v1}, UV{u0, v1}});
    }
  }
  const auto cap = [&](int ring, bool top) {
    const double r_max = std::max(1e-9, radius_at(top ? 1.0 : 0.0, 0.0));
    const Vec3 c{0, 0, top ? length / 2 : -length / 2};
    const F3 n{0, 0, top ? 1.0F : -1.0F};
    for (int j = 0; j < segments; ++j) {
      const auto& a = grid[static_cast<std::size_t>(ring)][static_cast<std::size_t>(j)];
      const auto& b = grid[static_cast<std::size_t>(ring)][static_cast<std::size_t>(j + 1)];
      const auto uv = [&](const Vec3& p) {
        return UV{static_cast<float>(0.5 + p.x / (2 * r_max)), static_cast<float>(0.5 - p.y / (2 * r_max))};
      };
      if (top) {
        mesh->add({to_f(c), to_f(a), to_f(b)}, {n, n, n}, {UV{0.5F, 0.5F}, uv(a), uv(b)});
      } else {
        mesh->add({to_f(c), to_f(b), to_f(a)}, {n, n, n}, {UV{0.5F, 0.5F}, uv(b), uv(a)});
      }
    }
  };
  if (bottom_cap) {
    cap(0, false);
  }
  if (top_cap) {
    cap(rings, true);
  }
  mesh->finish();
  return mesh;
}

}  // namespace

std::shared_ptr<Mesh> make_frustum(double r0, double r1, double length, int segments, bool caps) {
  return make_revolved([&](double t, double) { return r0 + (r1 - r0) * t; }, length, segments, 1, caps && r0 > 0,
                       caps && r1 > 0);
}

std::shared_ptr<Mesh> make_cylinder(double radius, double length, int segments) {
  return make_frustum(radius, radius, length, segments);
}

std::shared_ptr<Mesh> make_crushed_cylinder(double radius, double length, double crush, std::uint32_t seed,
                                            int segments) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> phase(0, 2 * kPi);
  const double p1 = phase(rng), p2 = phase(rng);
  return make_revolved(
      [&](double t, double a) {
        // 가운데가 눌리고 각도에 따라 울퉁불퉁 (찌그러진 캔)
        const double pinch = std::sin(t * kPi);
        const double dent = 0.5 + 0.5 * std::sin(3 * a + p1) * std::cos(2 * a + p2);
        return radius * (1.0 - crush * pinch * (0.4 + 0.6 * dent));
      },
      length, segments, 10, true, true);
}

std::shared_ptr<Mesh> make_lumpy_sphere(double radius, double roughness, std::uint32_t seed, int rings, int segments) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> jitter(1.0 - roughness, 1.0 + roughness);
  std::vector<std::vector<Vec3>> grid(static_cast<std::size_t>(rings + 1),
                                      std::vector<Vec3>(static_cast<std::size_t>(segments)));
  for (int i = 0; i <= rings; ++i) {
    const double phi = kPi * i / rings;
    const double pole_r = radius * jitter(rng);
    for (int j = 0; j < segments; ++j) {
      const double theta = 2 * kPi * j / segments;
      const double rr = (i == 0 || i == rings) ? pole_r : radius * jitter(rng);
      grid[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = {
          rr * std::sin(phi) * std::cos(theta), rr * std::sin(phi) * std::sin(theta), rr * std::cos(phi)};
    }
  }
  auto mesh = std::make_shared<Mesh>();
  for (int i = 0; i < rings; ++i) {
    for (int j = 0; j < segments; ++j) {
      const auto j0 = static_cast<std::size_t>(j), j1 = static_cast<std::size_t>((j + 1) % segments);
      const Vec3& a = grid[static_cast<std::size_t>(i)][j0];
      const Vec3& b = grid[static_cast<std::size_t>(i + 1)][j0];
      const Vec3& c = grid[static_cast<std::size_t>(i + 1)][j1];
      const Vec3& d = grid[static_cast<std::size_t>(i)][j1];
      const auto uv = [&](int ii, int jj) {
        return UV{static_cast<float>(jj) / segments, static_cast<float>(ii) / rings};
      };
      // 구겨진 종이는 면이 각져 보이도록 면 법선을 쓴다
      const F3 n1 = to_f(normalized(cross(b - a, c - a)));
      const F3 n2 = to_f(normalized(cross(c - a, d - a)));
      mesh->add({to_f(a), to_f(b), to_f(c)}, {n1, n1, n1}, {uv(i, j), uv(i + 1, j), uv(i + 1, j + 1)});
      mesh->add({to_f(a), to_f(c), to_f(d)}, {n2, n2, n2}, {uv(i, j), uv(i + 1, j + 1), uv(i, j + 1)});
    }
  }
  mesh->finish();
  return mesh;
}

std::shared_ptr<Mesh> make_quad(double sx, double sy) {
  auto mesh = std::make_shared<Mesh>();
  const float x = static_cast<float>(sx / 2), y = static_cast<float>(sy / 2);
  const F3 n{0, 0, 1};
  mesh->add({F3{-x, -y, 0}, F3{x, -y, 0}, F3{x, y, 0}}, {n, n, n}, {UV{0, 1}, UV{1, 1}, UV{1, 0}});
  mesh->add({F3{-x, -y, 0}, F3{x, y, 0}, F3{-x, y, 0}}, {n, n, n}, {UV{0, 1}, UV{1, 0}, UV{0, 0}});
  mesh->finish();
  return mesh;
}

std::shared_ptr<Mesh> load_stl(const std::string& path, const std::array<double, 3>& scale) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("STL을 열 수 없다: " + path);
  }
  const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  auto mesh = std::make_shared<Mesh>();  // 위치만 (법선은 면 법선, 메모리 절약)
  const auto scaled = [&](double x, double y, double z) {
    return F3{static_cast<float>(x * scale[0]), static_cast<float>(y * scale[1]), static_cast<float>(z * scale[2])};
  };
  std::uint32_t count = 0;
  if (data.size() >= 84) {
    std::memcpy(&count, data.data() + 80, 4);
  }
  if (data.size() >= 84 && data.size() == 84 + static_cast<std::size_t>(count) * 50) {  // 이진 STL
    mesh->positions.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
      const char* p = data.data() + 84 + static_cast<std::size_t>(i) * 50 + 12;
      float v[9];
      std::memcpy(v, p, sizeof(v));
      mesh->positions.push_back({scaled(v[0], v[1], v[2]), scaled(v[3], v[4], v[5]), scaled(v[6], v[7], v[8])});
    }
  } else {  // 텍스트 STL
    std::istringstream text(data);
    std::string word;
    std::vector<F3> vertices;
    while (text >> word) {
      if (word == "vertex") {
        double x = 0, y = 0, z = 0;
        text >> x >> y >> z;
        vertices.push_back(scaled(x, y, z));
        if (vertices.size() == 3) {
          mesh->positions.push_back({vertices[0], vertices[1], vertices[2]});
          vertices.clear();
        }
      }
    }
  }
  mesh->finish();
  return mesh;
}

// ---- 카메라·래스터 ----

void Camera::set_fov(double horizontal_fov) {
  fx = (width / 2.0) / std::tan(horizontal_fov / 2.0);
  fy = fx;
  cx = width / 2.0;
  cy = height / 2.0;
}

namespace {

// 화면에 투영된 꼭짓점. 원근 보정을 위해 속성은 1/z를 곱해 둔다.
struct ScreenVertex {
  float x, y;     // 픽셀 (슈퍼샘플 해상도)
  float inv_z;
  float u_z, v_z;           // uv / z
  float nx_z, ny_z, nz_z;   // 카메라 좌표 법선 / z
  float px_z, py_z, pz_z;   // 카메라 좌표 위치 / z (광택용 시선)
};

struct ScreenTriangle {
  ScreenVertex v[3];
  const Material* material;
  float depth_key;
  int id;  // 투명 정렬 (가장 먼 z)
  int min_y, max_y;
};

struct ShadeContext {
  Vec3 key;   // 카메라 좌표, 표면 → 빛
  Vec3 fill;
  Vec3 up;    // 카메라 좌표 월드 +z
  const Lighting* lighting;
};

std::array<float, 4> shade(const ScreenTriangle& tri, float w0, float w1, float w2, const ShadeContext& ctx) {
  const ScreenVertex& a = tri.v[0];
  const ScreenVertex& b = tri.v[1];
  const ScreenVertex& c = tri.v[2];
  const float inv_z = w0 * a.inv_z + w1 * b.inv_z + w2 * c.inv_z;
  const float z = 1.0F / inv_z;
  const auto attr = [&](float ScreenVertex::*field) {
    return (w0 * (a.*field) + w1 * (b.*field) + w2 * (c.*field)) * z;
  };
  const Material& m = *tri.material;
  std::array<float, 4> albedo{m.color[0], m.color[1], m.color[2], m.opacity};
  if (m.texture) {
    const float u = attr(&ScreenVertex::u_z) * m.uv_scale[0] + m.uv_offset[0];
    const float v = attr(&ScreenVertex::v_z) * m.uv_scale[1] + m.uv_offset[1];
    const auto t = m.texture->sample(u, v);
    albedo = {albedo[0] * t[0], albedo[1] * t[1], albedo[2] * t[2], albedo[3] * t[3]};
  }
  if (m.blend == Blend::Multiply || m.emissive >= 1.0F) {
    return albedo;
  }
  Vec3 n = normalized({attr(&ScreenVertex::nx_z), attr(&ScreenVertex::ny_z), attr(&ScreenVertex::nz_z)});
  const Vec3 p{attr(&ScreenVertex::px_z), attr(&ScreenVertex::py_z), attr(&ScreenVertex::pz_z)};
  const Vec3 view = normalized(p * -1.0);
  if (dot(n, view) < 0) {
    n = n * -1.0;  // 양면
  }
  const Lighting& l = *ctx.lighting;
  const double up = dot(n, ctx.up) * 0.5 + 0.5;
  const double kd = l.key * std::max(0.0, dot(n, ctx.key)) + l.fill * std::max(0.0, dot(n, ctx.fill));
  const Vec3 h = normalized(ctx.key + view);
  const double spec = m.specular * std::pow(std::max(0.0, dot(n, h)), static_cast<double>(m.shininess)) * l.key * 1.6;
  std::array<float, 4> out{};
  for (int i = 0; i < 3; ++i) {
    const double ambient = l.ground[static_cast<std::size_t>(i)] * (1 - up) + l.sky[static_cast<std::size_t>(i)] * up;
    const double lit = albedo[static_cast<std::size_t>(i)] * (ambient + kd) + spec;
    out[static_cast<std::size_t>(i)] =
        static_cast<float>(lit * (1 - m.emissive) + albedo[static_cast<std::size_t>(i)] * m.emissive);
  }
  out[3] = albedo[3];
  return out;
}

void raster_band(const std::vector<ScreenTriangle>& tris, int y_begin, int y_end, int width, std::vector<float>& zbuf,
                 std::vector<float>& color, std::vector<std::int32_t>* ids, const ShadeContext& ctx, bool transparent) {
  for (const auto& tri : tris) {
    if (tri.max_y < y_begin || tri.min_y >= y_end) {
      continue;
    }
    const ScreenVertex& a = tri.v[0];
    const ScreenVertex& b = tri.v[1];
    const ScreenVertex& c = tri.v[2];
    const float area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    if (std::abs(area) < 1e-9F) {
      continue;
    }
    const float inv_area = 1.0F / area;
    const int x0 = std::max(0, static_cast<int>(std::floor(std::min({a.x, b.x, c.x}))));
    const int x1 = std::min(width - 1, static_cast<int>(std::ceil(std::max({a.x, b.x, c.x}))));
    const int y0 = std::max(y_begin, tri.min_y);
    const int y1 = std::min(y_end - 1, tri.max_y);
    for (int y = y0; y <= y1; ++y) {
      const float py = static_cast<float>(y) + 0.5F;
      for (int x = x0; x <= x1; ++x) {
        const float px = static_cast<float>(x) + 0.5F;
        const float w0 = ((b.x - px) * (c.y - py) - (b.y - py) * (c.x - px)) * inv_area;
        const float w1 = ((c.x - px) * (a.y - py) - (c.y - py) * (a.x - px)) * inv_area;
        const float w2 = 1.0F - w0 - w1;
        if (w0 < 0 || w1 < 0 || w2 < 0) {
          continue;
        }
        const float inv_z = w0 * a.inv_z + w1 * b.inv_z + w2 * c.inv_z;
        if (inv_z <= 0) {
          continue;
        }
        const float z = 1.0F / inv_z;
        const std::size_t index = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                                  static_cast<std::size_t>(x);
        if (z >= zbuf[index]) {
          continue;
        }
        const auto rgba = shade(tri, w0, w1, w2, ctx);
        float* dst = &color[index * 3];
        if (!transparent) {
          if (rgba[3] < 0.5F) {
            continue;  // 잘라낸 부분 (알파 테스트)
          }
          zbuf[index] = z;
          if (ids != nullptr) (*ids)[index] = tri.id;
          dst[0] = rgba[0];
          dst[1] = rgba[1];
          dst[2] = rgba[2];
        } else if (tri.material->blend == Blend::Multiply) {
          const float k = 1.0F - rgba[3] * tri.material->opacity;
          dst[0] *= k;
          dst[1] *= k;
          dst[2] *= k;
        } else {
          const float alpha = rgba[3];
          if (ids != nullptr && alpha >= 0.25F && tri.id >= 0) (*ids)[index] = tri.id;  // 투명 물체도 보이면 번호
          dst[0] = dst[0] * (1 - alpha) + rgba[0] * alpha;
          dst[1] = dst[1] * (1 - alpha) + rgba[1] * alpha;
          dst[2] = dst[2] * (1 - alpha) + rgba[2] * alpha;
        }
      }
    }
  }
}

}  // namespace

void render(const Camera& base_camera, const std::vector<Instance>& instances, const Lighting& lighting, Frame& frame,
            const RenderOptions& options) {
  const int ss = std::max(1, options.supersample);
  Camera camera = base_camera;
  camera.width *= ss;
  camera.height *= ss;
  camera.fx *= ss;
  camera.fy *= ss;
  camera.cx *= ss;
  camera.cy *= ss;
  const int width = camera.width, height = camera.height;
  const std::size_t pixels = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  std::vector<float> zbuf(pixels, std::numeric_limits<float>::infinity());
  std::vector<float> color(pixels * 3);
  std::vector<std::int32_t> id_buffer(options.ids ? pixels : 0, -1);
  std::vector<std::int32_t>* ids = options.ids ? &id_buffer : nullptr;
  for (std::size_t i = 0; i < pixels; ++i) {
    // 배경: 위로 갈수록 살짝 밝게
    const float t = 1.0F - static_cast<float>(i / static_cast<std::size_t>(width)) / static_cast<float>(height);
    for (int k = 0; k < 3; ++k) {
      color[i * 3 + static_cast<std::size_t>(k)] = lighting.background[static_cast<std::size_t>(k)] * (0.85F + 0.25F * t);
    }
  }

  const Pose optical_from_world = camera.world_from_optical.inverse();
  ShadeContext ctx;
  ctx.key = normalized(optical_from_world.rotate(lighting.key_direction) * -1.0);
  ctx.fill = normalized(optical_from_world.rotate(lighting.fill_direction) * -1.0);
  ctx.up = optical_from_world.rotate({0, 0, 1});
  ctx.lighting = &lighting;
  const double tan_x = (width / 2.0) / camera.fx;
  const double tan_y = (height / 2.0) / camera.fy;

  // 1) 꼭짓점 변환·자르기·투영 → 화면 삼각형
  std::vector<ScreenTriangle> opaque;
  std::vector<ScreenTriangle> transparent;
  for (const auto& instance : instances) {
    const Mesh* mesh = instance.mesh.get();
    if (mesh == nullptr || mesh->positions.empty()) {
      continue;
    }
    const Pose optical_from_mesh = optical_from_world * instance.pose;
    const Vec3 c = optical_from_mesh.apply(mesh->center);
    const double r = mesh->radius;
    if (c.z + r < camera.near || c.z - r > camera.far ||
        std::abs(c.x) - r > std::max(c.z, 0.0) * tan_x + r || std::abs(c.y) - r > std::max(c.z, 0.0) * tan_y + r) {
      continue;
    }
    const bool is_transparent = instance.material.blend != Blend::Opaque;
    auto& out = is_transparent ? transparent : opaque;
    const bool has_normals = mesh->normals.size() == mesh->positions.size();
    const bool has_uvs = mesh->uvs.size() == mesh->positions.size();
    for (std::size_t t = 0; t < mesh->positions.size(); ++t) {
      Vec3 p[3];
      for (int k = 0; k < 3; ++k) {
        p[k] = optical_from_mesh.apply(to_d(mesh->positions[t][static_cast<std::size_t>(k)]));
      }
      if (p[0].z < camera.near && p[1].z < camera.near && p[2].z < camera.near) {
        continue;
      }
      if ((p[0].x > p[0].z * tan_x && p[1].x > p[1].z * tan_x && p[2].x > p[2].z * tan_x) ||
          (-p[0].x > p[0].z * tan_x && -p[1].x > p[1].z * tan_x && -p[2].x > p[2].z * tan_x) ||
          (p[0].y > p[0].z * tan_y && p[1].y > p[1].z * tan_y && p[2].y > p[2].z * tan_y) ||
          (-p[0].y > p[0].z * tan_y && -p[1].y > p[1].z * tan_y && -p[2].y > p[2].z * tan_y)) {
        continue;  // 화면 밖
      }
      Vec3 n[3];
      if (has_normals) {
        for (int k = 0; k < 3; ++k) {
          n[k] = optical_from_mesh.rotate(to_d(mesh->normals[t][static_cast<std::size_t>(k)]));
        }
      } else {
        n[0] = n[1] = n[2] = normalized(cross(p[1] - p[0], p[2] - p[0]));
      }
      UV uv[3]{};
      if (has_uvs) {
        for (int k = 0; k < 3; ++k) {
          uv[k] = mesh->uvs[t][static_cast<std::size_t>(k)];
        }
      }
      // near 평면 자르기 (속성도 같이 보간)
      struct Clip {
        Vec3 p, n;
        UV uv;
      };
      Clip in[3] = {{p[0], n[0], uv[0]}, {p[1], n[1], uv[1]}, {p[2], n[2], uv[2]}};
      Clip poly[4];
      int count = 0;
      for (int i = 0; i < 3; ++i) {
        const Clip& a = in[i];
        const Clip& b = in[(i + 1) % 3];
        const bool a_in = a.p.z >= camera.near, b_in = b.p.z >= camera.near;
        if (a_in) {
          poly[count++] = a;
        }
        if (a_in != b_in) {
          const double s = (camera.near - a.p.z) / (b.p.z - a.p.z);
          poly[count++] = {a.p + (b.p - a.p) * s, a.n + (b.n - a.n) * s,
                           UV{static_cast<float>(a.uv.u + (b.uv.u - a.uv.u) * s),
                              static_cast<float>(a.uv.v + (b.uv.v - a.uv.v) * s)}};
        }
      }
      ScreenVertex sv[4];
      float far_z = 0;
      for (int i = 0; i < count; ++i) {
        const Clip& q = poly[i];
        const float iz = static_cast<float>(1.0 / q.p.z);
        sv[i] = {static_cast<float>(camera.fx * q.p.x / q.p.z + camera.cx),
                 static_cast<float>(camera.fy * q.p.y / q.p.z + camera.cy),
                 iz,
                 q.uv.u * iz,
                 q.uv.v * iz,
                 static_cast<float>(q.n.x) * iz,
                 static_cast<float>(q.n.y) * iz,
                 static_cast<float>(q.n.z) * iz,
                 static_cast<float>(q.p.x) * iz,
                 static_cast<float>(q.p.y) * iz,
                 static_cast<float>(q.p.z) * iz};
        far_z = std::max(far_z, static_cast<float>(q.p.z));
      }
      for (int i = 1; i + 1 < count; ++i) {
        ScreenTriangle tri{{sv[0], sv[i], sv[i + 1]}, &instance.material, far_z, instance.id, 0, 0};
        const float min_y = std::min({tri.v[0].y, tri.v[1].y, tri.v[2].y});
        const float max_y = std::max({tri.v[0].y, tri.v[1].y, tri.v[2].y});
        if (max_y < 0 || min_y >= static_cast<float>(height)) {
          continue;
        }
        tri.min_y = std::max(0, static_cast<int>(std::floor(min_y)));
        tri.max_y = std::min(height - 1, static_cast<int>(std::ceil(max_y)));
        out.push_back(tri);
      }
    }
  }
  std::sort(transparent.begin(), transparent.end(),
            [](const ScreenTriangle& a, const ScreenTriangle& b) { return a.depth_key > b.depth_key; });

  // 2) 행 묶음을 스레드에 나눠 래스터 (불투명 → 투명)
  const int threads = std::clamp(options.threads, 1, 16);
  const auto run = [&](bool is_transparent) {
    const auto& list = is_transparent ? transparent : opaque;
    if (threads == 1) {
      raster_band(list, 0, height, width, zbuf, color, ids, ctx, is_transparent);
      return;
    }
    std::vector<std::thread> workers;
    const int band = (height + threads - 1) / threads;
    for (int i = 0; i < threads; ++i) {
      const int y0 = i * band, y1 = std::min(height, y0 + band);
      if (y0 >= y1) {
        break;
      }
      workers.emplace_back([&, y0, y1] {
        raster_band(list, y0, y1, width, zbuf, color, ids, ctx, is_transparent);
      });
    }
    for (auto& worker : workers) {
      worker.join();
    }
  };
  run(false);
  run(true);

  // 3) 슈퍼샘플 평균 → 출력, 깊이는 블록 안 최솟값, 비네팅·센서 잡음
  frame.width = base_camera.width;
  frame.height = base_camera.height;
  const std::size_t out_pixels = static_cast<std::size_t>(frame.width) * static_cast<std::size_t>(frame.height);
  frame.rgb.resize(out_pixels * 3);
  frame.depth_mm.assign(out_pixels, 0);
  frame.ids.assign(options.ids ? out_pixels : 0, -1);
  std::mt19937 rng(options.seed);
  std::normal_distribution<float> noise(0.0F, options.noise);
  const float inv = 1.0F / static_cast<float>(ss * ss);
  for (int y = 0; y < frame.height; ++y) {
    for (int x = 0; x < frame.width; ++x) {
      float sum[3] = {0, 0, 0};
      float zmin = std::numeric_limits<float>::infinity();
      std::int32_t id_near = -1;
      std::int32_t id_any = -1;  // 깊이가 없는 투명 물체만 있을 때
      for (int dy = 0; dy < ss; ++dy) {
        for (int dx = 0; dx < ss; ++dx) {
          const std::size_t i = static_cast<std::size_t>(y * ss + dy) * static_cast<std::size_t>(width) +
                                static_cast<std::size_t>(x * ss + dx);
          sum[0] += color[i * 3];
          sum[1] += color[i * 3 + 1];
          sum[2] += color[i * 3 + 2];
          if (zbuf[i] < zmin) {
            zmin = zbuf[i];
            if (ids != nullptr) id_near = id_buffer[i];  // 블록에서 가장 가까운 것
          }
          if (ids != nullptr && id_any < 0) id_any = id_buffer[i];
        }
      }
      const float nx = (static_cast<float>(x) + 0.5F) / static_cast<float>(frame.width) - 0.5F;
      const float ny = (static_cast<float>(y) + 0.5F) / static_cast<float>(frame.height) - 0.5F;
      const float vignette = 1.0F - options.vignette * (nx * nx + ny * ny) * 2.0F;
      const float grain = options.noise > 0 ? noise(rng) : 0.0F;
      const std::size_t o = static_cast<std::size_t>(y) * static_cast<std::size_t>(frame.width) + static_cast<std::size_t>(x);
      for (int k = 0; k < 3; ++k) {
        const float value = sum[k] * inv * vignette + grain;
        frame.rgb[o * 3 + static_cast<std::size_t>(k)] = static_cast<std::uint8_t>(std::clamp(value, 0.0F, 1.0F) * 255.0F + 0.5F);
      }
      if (std::isfinite(zmin) && zmin < camera.far) {
        frame.depth_mm[o] = static_cast<std::uint16_t>(std::min(65535.0F, zmin * 1000.0F));
      }
      if (ids != nullptr) frame.ids[o] = id_near >= 0 ? id_near : id_any;
    }
  }
}

// ---- URDF 로봇 ----

RobotMesh::RobotMesh(const std::string& urdf_path) {
  std::ifstream file(urdf_path);
  if (!file) {
    throw std::runtime_error("URDF를 열 수 없다: " + urdf_path);
  }
  std::stringstream text;
  text << file.rdbuf();
  robot_ = srf::parse_urdf(text.str());
  urdf_dir_ = std::filesystem::path(urdf_path).parent_path().string();
  std::map<std::string, std::shared_ptr<const Mesh>> cache;
  for (const auto& link : robot_.links) {
    for (const auto& visual : link.visuals) {
      std::shared_ptr<const Mesh> mesh;
      const auto& g = visual.geometry;
      switch (g.type) {
        case srf::UrdfGeometryType::Mesh: {
          const std::string key = g.filename + "@" + std::to_string(g.scale[0]);
          auto& cached = cache[key];
          if (!cached) {
            cached = load_stl(resolve_mesh_path(urdf_dir_, g.filename), g.scale);
          }
          mesh = cached;
          break;
        }
        case srf::UrdfGeometryType::Box: mesh = make_box(g.size[0], g.size[1], g.size[2]); break;
        case srf::UrdfGeometryType::Cylinder: mesh = make_cylinder(g.radius, g.length); break;
        case srf::UrdfGeometryType::Sphere: mesh = make_lumpy_sphere(g.radius, 0.0, 0); break;
        case srf::UrdfGeometryType::None: continue;
      }
      Material material;
      material.color = {0.6F, 0.6F, 0.6F};
      material.specular = 0.25F;  // 플라스틱·알루미늄 부품
      material.shininess = 40.0F;
      if (const auto found = robot_.materials.find(visual.material); found != robot_.materials.end()) {
        material.color = {static_cast<float>(found->second[0]), static_cast<float>(found->second[1]),
                          static_cast<float>(found->second[2])};
      }
      visuals_.push_back({link.name, Pose::rpy(visual.xyz, visual.rpy), mesh, material});
    }
  }
}

std::string resolve_mesh_path(const std::string& urdf_dir, const std::string& filename) {
  // package://<패키지>/<경로> → URDF가 <패키지>/urdf/ 안에 있으면 그 위, 아니면 형제 폴더 <패키지>.
  constexpr std::string_view kPackage = "package://";
  constexpr std::string_view kFile = "file://";
  if (filename.rfind(kFile, 0) == 0) {
    return filename.substr(kFile.size());
  }
  if (filename.rfind(kPackage, 0) != 0) {
    return (std::filesystem::path(urdf_dir) / filename).string();
  }
  const std::string rest = filename.substr(kPackage.size());
  const std::size_t slash = rest.find('/');
  const std::string package = rest.substr(0, slash);
  const std::string relative = slash == std::string::npos ? std::string{} : rest.substr(slash + 1);
  const std::filesystem::path package_root = std::filesystem::path(urdf_dir).parent_path();
  if (package_root.filename() == package) {
    return (package_root / relative).string();
  }
  return (package_root.parent_path() / package / relative).string();
}

std::map<std::string, Pose> forward_kinematics(const srf::UrdfRobot& robot, const srf::msgs::Pose2D& base,
                                               const std::map<std::string, double>& joints) {
  std::map<std::string, Pose> poses;
  poses[robot.root] = Pose::translation(base.x, base.y, 0.0) * Pose::yaw(base.theta);
  bool progressed = true;
  while (progressed) {
    progressed = false;
    for (const auto& joint : robot.joints) {
      if (poses.count(joint.child) != 0 || poses.count(joint.parent) == 0) {
        continue;
      }
      Pose pose = poses[joint.parent] * Pose::rpy(joint.xyz, joint.rpy);
      const auto q = joints.find(joint.name);
      const double value = q == joints.end() ? 0.0 : q->second;
      if (joint.type == "revolute" || joint.type == "continuous") {
        pose = pose * Pose::axis_angle(joint.axis, value);
      } else if (joint.type == "prismatic") {
        pose = pose * Pose::translation(joint.axis[0] * value, joint.axis[1] * value, joint.axis[2] * value);
      }
      poses[joint.child] = pose;
      progressed = true;
    }
  }
  return poses;
}

void RobotMesh::add_instances(const srf::msgs::Pose2D& base, const std::map<std::string, double>& joints,
                              std::vector<Instance>& out) const {
  const auto poses = forward_kinematics(robot_, base, joints);
  for (const auto& visual : visuals_) {
    if (std::find(hidden_.begin(), hidden_.end(), visual.link) != hidden_.end()) {
      continue;
    }
    const auto found = poses.find(visual.link);
    if (found != poses.end()) {
      out.push_back({visual.mesh, found->second * visual.offset, visual.material});
    }
  }
}

Pose RobotMesh::link_pose(const std::string& link, const srf::msgs::Pose2D& base,
                          const std::map<std::string, double>& joints) const {
  const auto poses = forward_kinematics(robot_, base, joints);
  const auto found = poses.find(link);
  if (found == poses.end()) {
    throw std::runtime_error("URDF에 링크가 없다: " + link);
  }
  return found->second;
}

void RobotMesh::hide_link(const std::string& link) { hidden_.push_back(link); }

std::size_t RobotMesh::triangle_count() const {
  std::size_t total = 0;
  for (const auto& visual : visuals_) {
    total += visual.mesh ? visual.mesh->triangle_count() : 0;
  }
  return total;
}

}  // namespace srf_recycling_ws::sim::render
