#include "krosc/urdf.hpp"

#include <cctype>
#include <cstdlib>
#include <memory>
#include <set>
#include <sstream>
#include <utility>

namespace krosc {

namespace {

struct XmlNode {
  std::string name;
  std::map<std::string, std::string> attributes;
  std::vector<std::unique_ptr<XmlNode>> children;
};

void skip_space(std::string_view text, std::size_t& pos) {
  while (pos < text.size() && std::isspace(static_cast<unsigned char>(text[pos])) != 0) {
    ++pos;
  }
}

void skip_comment(std::string_view text, std::size_t& pos) {
  const std::size_t end = text.find("-->", pos + 4);
  if (end == std::string_view::npos) {
    throw UrdfError("XML 주석이 닫히지 않았다");
  }
  pos = end + 3;
}

void skip_until(std::string_view text, std::size_t& pos, std::string_view terminator, const char* what) {
  const std::size_t end = text.find(terminator, pos);
  if (end == std::string_view::npos) {
    throw UrdfError(std::string(what) + "이(가) 닫히지 않았다");
  }
  pos = end + terminator.size();
}

std::string decode_entities(std::string_view value) {
  std::string out;
  out.reserve(value.size());
  for (std::size_t i = 0; i < value.size(); ++i) {
    if (value[i] != '&') {
      out += value[i];
      continue;
    }
    const std::size_t semi = value.find(';', i + 1);
    if (semi == std::string_view::npos) {
      out += value[i];
      continue;
    }
    const std::string_view entity = value.substr(i + 1, semi - i - 1);
    if (entity == "amp") {
      out += '&';
    } else if (entity == "lt") {
      out += '<';
    } else if (entity == "gt") {
      out += '>';
    } else if (entity == "quot") {
      out += '"';
    } else if (entity == "apos") {
      out += '\'';
    } else {
      out += value.substr(i, semi - i + 1);
    }
    i = semi;
  }
  return out;
}

std::string parse_name(std::string_view text, std::size_t& pos) {
  const std::size_t start = pos;
  while (pos < text.size()) {
    const char c = text[pos];
    if (std::isspace(static_cast<unsigned char>(c)) != 0 || c == '/' || c == '>') {
      break;
    }
    ++pos;
  }
  if (pos == start) {
    throw UrdfError("XML 태그 이름이 비었다");
  }
  return std::string(text.substr(start, pos - start));
}

XmlNode parse_xml(std::string_view text) {
  std::size_t pos = 0;
  std::vector<XmlNode*> stack;
  XmlNode document;
  document.name = "#document";
  stack.push_back(&document);

  while (pos < text.size()) {
    if (text[pos] != '<') {
      ++pos;  // 텍스트는 쓰지 않는다
      continue;
    }
    if (text.compare(pos, 4, "<!--") == 0) {
      skip_comment(text, pos);
      continue;
    }
    if (text.compare(pos, 2, "<?") == 0) {
      skip_until(text, pos, "?>", "XML 선언");
      continue;
    }
    if (text.compare(pos, 2, "<!") == 0) {
      skip_until(text, pos, ">", "DOCTYPE");
      continue;
    }
    if (text.compare(pos, 2, "</") == 0) {
      pos += 2;
      skip_space(text, pos);
      const std::string name = parse_name(text, pos);
      skip_space(text, pos);
      if (pos >= text.size() || text[pos] != '>') {
        throw UrdfError("XML 닫는 태그가 올바르지 않다: " + name);
      }
      ++pos;
      if (stack.size() <= 1 || stack.back()->name != name) {
        throw UrdfError("XML 태그 짝이 맞지 않다: " + name);
      }
      stack.pop_back();
      continue;
    }

    ++pos;  // '<'
    skip_space(text, pos);
    const std::string name = parse_name(text, pos);
    auto node = std::make_unique<XmlNode>();
    node->name = name;
    bool self_closing = false;
    while (pos < text.size()) {
      skip_space(text, pos);
      if (pos >= text.size()) {
        break;
      }
      if (text[pos] == '/') {
        ++pos;
        if (pos < text.size() && text[pos] == '>') {
          ++pos;
          self_closing = true;
        }
        break;
      }
      if (text[pos] == '>') {
        ++pos;
        break;
      }
      const std::size_t key_start = pos;
      while (pos < text.size() && text[pos] != '=' && std::isspace(static_cast<unsigned char>(text[pos])) == 0 &&
             text[pos] != '>' && text[pos] != '/') {
        ++pos;
      }
      const std::string key(text.substr(key_start, pos - key_start));
      skip_space(text, pos);
      std::string value;
      if (pos < text.size() && text[pos] == '=') {
        ++pos;
        skip_space(text, pos);
        if (pos >= text.size() || (text[pos] != '"' && text[pos] != '\'')) {
          throw UrdfError("XML 속성값이 따옴표로 시작하지 않는다: " + name + "." + key);
        }
        const char quote = text[pos++];
        const std::size_t value_start = pos;
        while (pos < text.size() && text[pos] != quote) {
          ++pos;
        }
        if (pos >= text.size()) {
          throw UrdfError("XML 속성값이 닫히지 않았다: " + name + "." + key);
        }
        value = decode_entities(text.substr(value_start, pos - value_start));
        ++pos;
      }
      if (!key.empty()) {
        node->attributes[key] = value;
      } else {
        ++pos;
      }
    }
    XmlNode* raw = node.get();
    stack.back()->children.push_back(std::move(node));
    if (!self_closing) {
      stack.push_back(raw);
    }
  }
  if (stack.size() != 1) {
    throw UrdfError("XML 태그가 닫히지 않았다: " + stack.back()->name);
  }
  if (document.children.size() != 1) {
    throw UrdfError("XML 최상위 요소가 하나가 아니다");
  }
  XmlNode root = std::move(*document.children.front());
  return root;
}

const XmlNode* child(const XmlNode& node, const std::string& name) {
  for (const auto& item : node.children) {
    if (item->name == name) {
      return item.get();
    }
  }
  return nullptr;
}

std::vector<const XmlNode*> children(const XmlNode& node, const std::string& name) {
  std::vector<const XmlNode*> result;
  for (const auto& item : node.children) {
    if (item->name == name) {
      result.push_back(item.get());
    }
  }
  return result;
}

const std::string& attribute(const XmlNode& node, const std::string& name, bool required = false) {
  static const std::string empty;
  auto it = node.attributes.find(name);
  if (it == node.attributes.end()) {
    if (required) {
      throw UrdfError("<" + node.name + ">에 " + name + " 속성이 없다");
    }
    return empty;
  }
  return it->second;
}

std::array<double, 3> floats3(const std::string& text, const std::array<double, 3>& fallback, const std::string& where) {
  if (text.empty()) {
    return fallback;
  }
  std::istringstream stream(text);
  std::array<double, 3> result{};
  if (!(stream >> result[0] >> result[1] >> result[2])) {
    throw UrdfError(where + ": 숫자 3개가 아니다: " + text);
  }
  std::string extra;
  if (stream >> extra) {
    throw UrdfError(where + ": 숫자가 너무 많다: " + text);
  }
  return result;
}

double number(const std::string& text, const std::string& where) {
  if (text.empty()) {
    throw UrdfError(where + ": 값이 없다");
  }
  char* end = nullptr;
  const double value = std::strtod(text.c_str(), &end);
  if (end == text.c_str() || *end != '\0') {
    throw UrdfError(where + ": 숫자가 아니다: " + text);
  }
  return value;
}

UrdfGeometry parse_geometry(const XmlNode& geometry, const std::string& where) {
  UrdfGeometry result;
  if (const XmlNode* box = child(geometry, "box"); box != nullptr) {
    result.type = UrdfGeometryType::Box;
    result.size = floats3(attribute(*box, "size"), {0.0, 0.0, 0.0}, where + " box size");
  } else if (const XmlNode* cylinder = child(geometry, "cylinder"); cylinder != nullptr) {
    result.type = UrdfGeometryType::Cylinder;
    result.radius = number(attribute(*cylinder, "radius"), where + " cylinder radius");
    result.length = number(attribute(*cylinder, "length"), where + " cylinder length");
  } else if (const XmlNode* sphere = child(geometry, "sphere"); sphere != nullptr) {
    result.type = UrdfGeometryType::Sphere;
    result.radius = number(attribute(*sphere, "radius"), where + " sphere radius");
  } else if (const XmlNode* mesh = child(geometry, "mesh"); mesh != nullptr) {
    result.type = UrdfGeometryType::Mesh;
    result.filename = attribute(*mesh, "filename");
    result.scale = floats3(attribute(*mesh, "scale"), {1.0, 1.0, 1.0}, where + " mesh scale");
  }
  return result;
}

std::vector<UrdfVisual> parse_visuals(const XmlNode& link, const std::string& tag, const std::string& where) {
  std::vector<UrdfVisual> result;
  for (const XmlNode* item : children(link, tag)) {
    UrdfVisual visual;
    if (const XmlNode* origin = child(*item, "origin"); origin != nullptr) {
      visual.xyz = floats3(attribute(*origin, "xyz"), {0.0, 0.0, 0.0}, where + " origin xyz");
      visual.rpy = floats3(attribute(*origin, "rpy"), {0.0, 0.0, 0.0}, where + " origin rpy");
    }
    if (const XmlNode* geometry = child(*item, "geometry"); geometry != nullptr) {
      visual.geometry = parse_geometry(*geometry, where);
    }
    if (const XmlNode* material = child(*item, "material"); material != nullptr) {
      visual.material = attribute(*material, "name");
    }
    result.push_back(std::move(visual));
  }
  return result;
}

}  // namespace

UrdfRobot parse_urdf(std::string_view text, const std::map<std::string, std::string>& xacro_args) {
  std::string source(text);
  for (const auto& [key, value] : xacro_args) {
    const std::string pattern = "${" + key + "}";
    std::size_t pos = 0;
    while ((pos = source.find(pattern, pos)) != std::string::npos) {
      source.replace(pos, pattern.size(), value);
      pos += value.size();
    }
  }
  const XmlNode root = parse_xml(source);
  if (root.name != "robot") {
    throw UrdfError("최상위 태그가 <robot>이 아니다: <" + root.name + ">");
  }

  UrdfRobot robot;
  robot.name = attribute(root, "name");

  for (const XmlNode* material : children(root, "material")) {
    const std::string name = attribute(*material, "name");
    if (name.empty()) {
      continue;
    }
    std::array<double, 4> rgba{1.0, 1.0, 1.0, 1.0};
    if (const XmlNode* color = child(*material, "color"); color != nullptr) {
      const std::string text_value = attribute(*color, "rgba");
      std::istringstream stream(text_value);
      if (!(stream >> rgba[0] >> rgba[1] >> rgba[2] >> rgba[3])) {
        throw UrdfError("material " + name + "의 color rgba를 읽을 수 없다: " + text_value);
      }
    }
    robot.materials[name] = rgba;
  }

  for (const XmlNode* link : children(root, "link")) {
    const std::string name = attribute(*link, "name");
    if (name.empty()) {
      throw UrdfError("이름 없는 <link>가 있다");
    }
    if (robot.find_link(name) != nullptr) {
      throw UrdfError("링크 이름이 겹친다: " + name);
    }
    UrdfLink parsed;
    parsed.name = name;
    parsed.visuals = parse_visuals(*link, "visual", "링크 " + name);
    parsed.collisions = parse_visuals(*link, "collision", "링크 " + name);
    robot.links.push_back(std::move(parsed));
  }

  static const std::array<const char*, 6> joint_types = {"fixed", "continuous", "revolute",
                                                         "prismatic", "floating", "planar"};
  for (const XmlNode* joint : children(root, "joint")) {
    UrdfJoint parsed;
    parsed.name = attribute(*joint, "name");
    parsed.type = attribute(*joint, "type");
    if (parsed.name.empty()) {
      throw UrdfError("이름 없는 <joint>가 있다");
    }
    bool known_type = false;
    for (const char* type : joint_types) {
      known_type = known_type || parsed.type == type;
    }
    if (!known_type) {
      throw UrdfError("관절 " + parsed.name + "의 type이 올바르지 않다: " + parsed.type);
    }
    const XmlNode* parent = child(*joint, "parent");
    const XmlNode* child_link = child(*joint, "child");
    if (parent == nullptr || child_link == nullptr) {
      throw UrdfError("관절 " + parsed.name + "에 <parent>/<child>가 없다");
    }
    parsed.parent = attribute(*parent, "link");
    parsed.child = attribute(*child_link, "link");
    if (parsed.parent.empty() || parsed.child.empty()) {
      throw UrdfError("관절 " + parsed.name + "에 parent link/child link가 없다");
    }
    if (const XmlNode* origin = child(*joint, "origin"); origin != nullptr) {
      parsed.xyz = floats3(attribute(*origin, "xyz"), {0.0, 0.0, 0.0}, "관절 " + parsed.name + " origin xyz");
      parsed.rpy = floats3(attribute(*origin, "rpy"), {0.0, 0.0, 0.0}, "관절 " + parsed.name + " origin rpy");
    }
    if (const XmlNode* axis = child(*joint, "axis"); axis != nullptr) {
      parsed.axis = floats3(attribute(*axis, "xyz"), {1.0, 0.0, 0.0}, "관절 " + parsed.name + " axis");
    }
    robot.joints.push_back(std::move(parsed));
  }

  if (robot.links.empty()) {
    throw UrdfError("<link>가 하나도 없다");
  }
  std::map<std::string, std::string> parent_of;
  std::set<std::string> joint_names;
  for (const auto& joint : robot.joints) {
    if (!joint_names.insert(joint.name).second) {
      throw UrdfError("관절 이름이 겹친다: " + joint.name);
    }
    if (robot.find_link(joint.parent) == nullptr) {
      throw UrdfError("관절 " + joint.name + "이 없는 링크를 가리킨다: " + joint.parent);
    }
    if (robot.find_link(joint.child) == nullptr) {
      throw UrdfError("관절 " + joint.name + "이 없는 링크를 가리킨다: " + joint.child);
    }
    if (parent_of.count(joint.child) != 0) {
      throw UrdfError("링크 " + joint.child + "의 부모가 둘 이상이다");
    }
    parent_of[joint.child] = joint.parent;
  }
  std::vector<std::string> roots;
  for (const auto& link : robot.links) {
    if (parent_of.count(link.name) == 0) {
      roots.push_back(link.name);
    }
  }
  if (roots.size() != 1) {
    throw UrdfError("뿌리 링크(부모가 없는 링크)가 정확히 하나여야 한다");
  }
  robot.root = roots.front();
  for (const auto& link : robot.links) {
    std::string node = link.name;
    std::size_t hops = 0;
    while (parent_of.count(node) != 0) {
      node = parent_of[node];
      if (++hops > robot.links.size()) {
        throw UrdfError("링크 " + link.name + "에서 관절 순환이 있다");
      }
    }
    if (node != robot.root) {
      throw UrdfError("링크 " + link.name + "이 뿌리 " + robot.root + "에 이어져 있지 않다");
    }
    if (link.name.find("${") != std::string::npos) {
      throw UrdfError("치환하지 못한 xacro 표현이 이름에 남았다: " + link.name);
    }
  }
  for (const auto& joint : robot.joints) {
    if (joint.name.find("${") != std::string::npos) {
      throw UrdfError("치환하지 못한 xacro 표현이 이름에 남았다: " + joint.name);
    }
  }
  return robot;
}

}  // namespace krosc
