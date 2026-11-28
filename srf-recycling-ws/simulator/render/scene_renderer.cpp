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
  // URDF: R = Rz(yaw) * Ry(pitch) * Rx(roll)
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

// ---- 메시 ----

void Mesh::finish() {
  if (triangles.empty()) {
    return;
  }
  Vec3 low{1e9, 1e9, 1e9};
  Vec3 high{-1e9, -1e9, -1e9};
  for (const auto& tri : triangles) {
    for (const auto& v : tri) {
      low = {std::min(low.x, v.x), std::min(low.y, v.y), std::min(low.z, v.z)};
      high = {std::max(high.x, v.x), std::max(high.y, v.y), std::max(high.z, v.z)};
    }
  }
  center = (low + high) * 0.5;
  radius = std::sqrt(dot(high - center, high - center));
}

std::shared_ptr<Mesh> make_box(double sx, double sy, double sz) {
  auto mesh = std::make_shared<Mesh>();
  const double x = sx / 2, y = sy / 2, z = sz / 2;
  const Vec3 p[8] = {{-x, -y, -z}, {x, -y, -z}, {x, y, -z}, {-x, y, -z},
                     {-x, -y, z},  {x, -y, z},  {x, y, z},  {-x, y, z}};
  const int faces[6][4] = {{0, 1, 2, 3}, {4, 7, 6, 5}, {0, 4, 5, 1}, {1, 5, 6, 2}, {2, 6, 7, 3}, {3, 7, 4, 0}};
  for (const auto& f : faces) {
    mesh->triangles.push_back({p[f[0]], p[f[1]], p[f[2]]});
    mesh->triangles.push_back({p[f[0]], p[f[2]], p[f[3]]});
  }
  mesh->finish();
  return mesh;
}

std::shared_ptr<Mesh> make_frustum(double r0, double r1, double length, int segments) {
  auto mesh = std::make_shared<Mesh>();
  const double z0 = -length / 2, z1 = length / 2;
  for (int i = 0; i < segments; ++i) {
    const double a0 = 2 * kPi * i / segments, a1 = 2 * kPi * (i + 1) / segments;
    const Vec3 b0{r0 * std::cos(a0), r0 * std::sin(a0), z0}, b1{r0 * std::cos(a1), r0 * std::sin(a1), z0};
    const Vec3 t0{r1 * std::cos(a0), r1 * std::sin(a0), z1}, t1{r1 * std::cos(a1), r1 * std::sin(a1), z1};
    mesh->triangles.push_back({b0, b1, t1});
    mesh->triangles.push_back({b0, t1, t0});
    if (r0 > 0) {
      mesh->triangles.push_back({Vec3{0, 0, z0}, b1, b0});
    }
    if (r1 > 0) {
      mesh->triangles.push_back({Vec3{0, 0, z1}, t0, t1});
    }
  }
  mesh->finish();
  return mesh;
}

std::shared_ptr<Mesh> make_cylinder(double radius, double length, int segments) {
  return make_frustum(radius, radius, length, segments);
}

std::shared_ptr<Mesh> make_lumpy_sphere(double radius, double roughness, std::uint32_t seed, int rings, int segments) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> jitter(1.0 - roughness, 1.0 + roughness);
  // 격자 점마다 반지름을 흔든다. 극점은 하나로 묶는다.
  std::vector<std::vector<Vec3>> grid(static_cast<std::size_t>(rings + 1));
  for (int i = 0; i <= rings; ++i) {
    const double phi = kPi * i / rings;
    const int count = (i == 0 || i == rings) ? 1 : segments;
    for (int j = 0; j < segments; ++j) {
      const double theta = 2 * kPi * j / segments;
      const double rr = radius * (count == 1 && j > 0 ? 1.0 : jitter(rng));
      if (count == 1 && j > 0) {
        grid[static_cast<std::size_t>(i)].push_back(grid[static_cast<std::size_t>(i)][0]);
        continue;
      }
      grid[static_cast<std::size_t>(i)].push_back(
          {rr * std::sin(phi) * std::cos(theta), rr * std::sin(phi) * std::sin(theta), rr * std::cos(phi)});
    }
  }
  auto mesh = std::make_shared<Mesh>();
  for (int i = 0; i < rings; ++i) {
    for (int j = 0; j < segments; ++j) {
      const auto& a = grid[static_cast<std::size_t>(i)];
      const auto& b = grid[static_cast<std::size_t>(i + 1)];
      const auto j1 = static_cast<std::size_t>((j + 1) % segments);
      const auto j0 = static_cast<std::size_t>(j);
      mesh->triangles.push_back({a[j0], b[j0], b[j1]});
      mesh->triangles.push_back({a[j0], b[j1], a[j1]});
    }
  }
  mesh->finish();
  return mesh;
}

std::shared_ptr<Mesh> load_stl(const std::string& path, const std::array<double, 3>& scale) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("STL을 열 수 없다: " + path);
  }
  const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  auto mesh = std::make_shared<Mesh>();
  const auto scaled = [&](double x, double y, double z) { return Vec3{x * scale[0], y * scale[1], z * scale[2]}; };
  std::uint32_t count = 0;
  if (data.size() >= 84) {
    std::memcpy(&count, data.data() + 80, 4);
  }
  if (data.size() >= 84 && data.size() == 84 + static_cast<std::size_t>(count) * 50) {  // 이진 STL
    mesh->triangles.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
      const char* p = data.data() + 84 + static_cast<std::size_t>(i) * 50 + 12;  // 법선은 건너뛴다
      float v[9];
      std::memcpy(v, p, sizeof(v));
      mesh->triangles.push_back({scaled(v[0], v[1], v[2]), scaled(v[3], v[4], v[5]), scaled(v[6], v[7], v[8])});
    }
  } else {  // 텍스트 STL
    std::istringstream text(data);
    std::string word;
    std::vector<Vec3> vertices;
    while (text >> word) {
      if (word == "vertex") {
        double x = 0, y = 0, z = 0;
        text >> x >> y >> z;
        vertices.push_back(scaled(x, y, z));
        if (vertices.size() == 3) {
          mesh->triangles.push_back({vertices[0], vertices[1], vertices[2]});
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
  fy = fx;  // 정사각 픽셀
  cx = width / 2.0;
  cy = height / 2.0;
}

namespace {

struct Projected {
  double u, v, inv_z;
};

void fill_triangle(const Projected& a, const Projected& b, const Projected& c, const std::array<std::uint8_t, 3>& rgb,
                   std::vector<float>& zbuffer, Frame& frame) {
  const double area = (b.u - a.u) * (c.v - a.v) - (b.v - a.v) * (c.u - a.u);
  if (std::abs(area) < 1e-12) {
    return;
  }
  const int x0 = std::max(0, static_cast<int>(std::floor(std::min({a.u, b.u, c.u}))));
  const int x1 = std::min(frame.width - 1, static_cast<int>(std::ceil(std::max({a.u, b.u, c.u}))));
  const int y0 = std::max(0, static_cast<int>(std::floor(std::min({a.v, b.v, c.v}))));
  const int y1 = std::min(frame.height - 1, static_cast<int>(std::ceil(std::max({a.v, b.v, c.v}))));
  if (x0 > x1 || y0 > y1) {
    return;
  }
  const double inv_area = 1.0 / area;
  for (int y = y0; y <= y1; ++y) {
    const double py = y + 0.5;
    for (int x = x0; x <= x1; ++x) {
      const double px = x + 0.5;
      const double w0 = ((b.u - px) * (c.v - py) - (b.v - py) * (c.u - px)) * inv_area;
      const double w1 = ((c.u - px) * (a.v - py) - (c.v - py) * (a.u - px)) * inv_area;
      const double w2 = 1.0 - w0 - w1;
      if (w0 < 0 || w1 < 0 || w2 < 0) {
        continue;
      }
      const double inv_z = w0 * a.inv_z + w1 * b.inv_z + w2 * c.inv_z;  // 1/z는 화면에서 선형
      if (inv_z <= 0) {
        continue;
      }
      const auto z = static_cast<float>(1.0 / inv_z);
      const std::size_t index = static_cast<std::size_t>(y) * static_cast<std::size_t>(frame.width) +
                                static_cast<std::size_t>(x);
      if (z >= zbuffer[index]) {
        continue;
      }
      zbuffer[index] = z;
      std::memcpy(&frame.rgb[index * 3], rgb.data(), 3);
    }
  }
}

}  // namespace

void render(const Camera& camera, const std::vector<Instance>& instances, const Lighting& lighting, Frame& frame) {
  frame.width = camera.width;
  frame.height = camera.height;
  const std::size_t pixels = static_cast<std::size_t>(camera.width) * static_cast<std::size_t>(camera.height);
  frame.rgb.resize(pixels * 3);
  frame.depth_mm.assign(pixels, 0);
  for (std::size_t i = 0; i < pixels; ++i) {
    frame.rgb[i * 3 + 0] = static_cast<std::uint8_t>(lighting.background[0] * 255.0F);
    frame.rgb[i * 3 + 1] = static_cast<std::uint8_t>(lighting.background[1] * 255.0F);
    frame.rgb[i * 3 + 2] = static_cast<std::uint8_t>(lighting.background[2] * 255.0F);
  }
  std::vector<float> zbuffer(pixels, std::numeric_limits<float>::infinity());

  const Pose optical_from_world = camera.world_from_optical.inverse();
  const Vec3 light = normalized(optical_from_world.rotate(lighting.direction) * -1.0);  // 표면 → 빛
  const double tan_x = (camera.width / 2.0) / camera.fx;
  const double tan_y = (camera.height / 2.0) / camera.fy;

  for (const auto& instance : instances) {
    if (!instance.mesh || instance.mesh->triangles.empty()) {
      continue;
    }
    const Pose optical_from_mesh = optical_from_world * instance.pose;
    // 경계 구 컬링: 카메라 뒤·화면 밖이면 건너뛴다.
    const Vec3 c = optical_from_mesh.apply(instance.mesh->center);
    const double r = instance.mesh->radius;
    if (c.z + r < camera.near || c.z - r > camera.far ||
        std::abs(c.x) - r > std::max(c.z, 0.0) * tan_x + r || std::abs(c.y) - r > std::max(c.z, 0.0) * tan_y + r) {
      continue;
    }
    for (const auto& tri : instance.mesh->triangles) {
      Vec3 v[3] = {optical_from_mesh.apply(tri[0]), optical_from_mesh.apply(tri[1]), optical_from_mesh.apply(tri[2])};
      if (v[0].z < camera.near && v[1].z < camera.near && v[2].z < camera.near) {
        continue;
      }
      // 면 음영 (양면: STL 감김 방향이 제각각이다).
      const Vec3 normal = normalized(cross(v[1] - v[0], v[2] - v[0]));
      const double shade = lighting.ambient + lighting.diffuse * std::abs(dot(normal, light));
      const std::array<std::uint8_t, 3> rgb{
          static_cast<std::uint8_t>(std::clamp(instance.color[0] * shade, 0.0, 1.0) * 255.0),
          static_cast<std::uint8_t>(std::clamp(instance.color[1] * shade, 0.0, 1.0) * 255.0),
          static_cast<std::uint8_t>(std::clamp(instance.color[2] * shade, 0.0, 1.0) * 255.0)};
      // near 평면으로 자른다 (최대 사각형 → 삼각형 둘).
      Vec3 poly[4];
      int n = 0;
      for (int i = 0; i < 3; ++i) {
        const Vec3& p = v[i];
        const Vec3& q = v[(i + 1) % 3];
        const bool p_in = p.z >= camera.near;
        const bool q_in = q.z >= camera.near;
        if (p_in) {
          poly[n++] = p;
        }
        if (p_in != q_in) {
          const double s = (camera.near - p.z) / (q.z - p.z);
          poly[n++] = p + (q - p) * s;
        }
      }
      Projected pr[4];
      for (int i = 0; i < n; ++i) {
        pr[i] = {camera.fx * poly[i].x / poly[i].z + camera.cx, camera.fy * poly[i].y / poly[i].z + camera.cy,
                 1.0 / poly[i].z};
      }
      for (int i = 1; i + 1 < n; ++i) {
        fill_triangle(pr[0], pr[i], pr[i + 1], rgb, zbuffer, frame);
      }
    }
  }
  for (std::size_t i = 0; i < pixels; ++i) {
    if (std::isfinite(zbuffer[i]) && zbuffer[i] < camera.far) {
      frame.depth_mm[i] = static_cast<std::uint16_t>(std::min(65535.0F, zbuffer[i] * 1000.0F));
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
            cached = load_stl(resolve(g.filename), g.scale);
          }
          mesh = cached;
          break;
        }
        case srf::UrdfGeometryType::Box: mesh = make_box(g.size[0], g.size[1], g.size[2]); break;
        case srf::UrdfGeometryType::Cylinder: mesh = make_cylinder(g.radius, g.length); break;
        case srf::UrdfGeometryType::Sphere: mesh = make_lumpy_sphere(g.radius, 0.0, 0); break;
        case srf::UrdfGeometryType::None: continue;
      }
      Color color{0.6F, 0.6F, 0.6F};
      if (const auto found = robot_.materials.find(visual.material); found != robot_.materials.end()) {
        color = {static_cast<float>(found->second[0]), static_cast<float>(found->second[1]),
                 static_cast<float>(found->second[2])};
      }
      visuals_.push_back({link.name, Pose::rpy(visual.xyz, visual.rpy), mesh, color});
    }
  }
}

std::string RobotMesh::resolve(const std::string& filename) const {
  // package://<패키지>/<경로> → URDF가 <패키지>/urdf/ 안에 있으면 그 위, 아니면 형제 폴더 <패키지>.
  constexpr std::string_view kPackage = "package://";
  constexpr std::string_view kFile = "file://";
  if (filename.rfind(kFile, 0) == 0) {
    return filename.substr(kFile.size());
  }
  if (filename.rfind(kPackage, 0) != 0) {
    return (std::filesystem::path(urdf_dir_) / filename).string();
  }
  const std::string rest = filename.substr(kPackage.size());
  const std::size_t slash = rest.find('/');
  const std::string package = rest.substr(0, slash);
  const std::string relative = slash == std::string::npos ? std::string{} : rest.substr(slash + 1);
  const std::filesystem::path package_root = std::filesystem::path(urdf_dir_).parent_path();
  if (package_root.filename() == package) {
    return (package_root / relative).string();
  }
  return (package_root.parent_path() / package / relative).string();
}

std::map<std::string, Pose> RobotMesh::forward(const srf::msgs::Pose2D& base,
                                               const std::map<std::string, double>& joints) const {
  std::map<std::string, Pose> poses;
  poses[robot_.root] = Pose::translation(base.x, base.y, 0.0) * Pose::yaw(base.theta);
  // 관절은 부모가 먼저 나오도록 여러 번 훑는다 (링크 수가 작다).
  bool progressed = true;
  while (progressed) {
    progressed = false;
    for (const auto& joint : robot_.joints) {
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
  const auto poses = forward(base, joints);
  for (const auto& visual : visuals_) {
    if (std::find(hidden_.begin(), hidden_.end(), visual.link) != hidden_.end()) {
      continue;
    }
    const auto found = poses.find(visual.link);
    if (found != poses.end()) {
      out.push_back({visual.mesh, found->second * visual.offset, visual.color});
    }
  }
}

Pose RobotMesh::link_pose(const std::string& link, const srf::msgs::Pose2D& base,
                          const std::map<std::string, double>& joints) const {
  const auto poses = forward(base, joints);
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
    total += visual.mesh ? visual.mesh->triangles.size() : 0;
  }
  return total;
}

}  // namespace srf_recycling_ws::sim::render
