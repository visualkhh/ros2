#include "krosc/nodes/map_io.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "krosc/math.hpp"

namespace krosc::nodes {

namespace {

constexpr double kFreeThresh = 0.196;
constexpr double kOccupiedThresh = 0.65;
constexpr int kWallPixel = 0;
constexpr int kFreePixel = 254;
constexpr int kUnknownPixel = 205;

std::string trim(std::string text) {
  const auto not_space = [](unsigned char character) { return std::isspace(character) == 0; };
  text.erase(text.begin(), std::find_if(text.begin(), text.end(), not_space));
  text.erase(std::find_if(text.rbegin(), text.rend(), not_space).base(), text.end());
  return text;
}

std::map<std::string, std::string> parse_yaml(const std::string& text) {
  std::map<std::string, std::string> values;
  std::istringstream stream(text);
  std::string line;
  while (std::getline(stream, line)) {
    const std::size_t comment = line.find('#');
    if (comment != std::string::npos) {
      line = line.substr(0, comment);
    }
    line = trim(line);
    if (line.empty()) {
      continue;
    }
    const std::size_t colon = line.find(':');
    if (colon == std::string::npos) {
      throw MapIoError("yaml 줄을 읽을 수 없다: " + line);
    }
    std::string value = trim(line.substr(colon + 1));
    if (value.size() >= 2 && ((value.front() == '\'' && value.back() == '\'') ||
                              (value.front() == '"' && value.back() == '"'))) {
      value = value.substr(1, value.size() - 2);
    }
    values[trim(line.substr(0, colon))] = value;
  }
  return values;
}

double as_double(const std::map<std::string, std::string>& values, const std::string& key, double fallback) {
  const auto found = values.find(key);
  if (found == values.end()) {
    return fallback;
  }
  try {
    return std::stod(found->second);
  } catch (...) {
    throw MapIoError("yaml 값이 숫자가 아니다: " + key + "=" + found->second);
  }
}

std::vector<double> as_list(const std::map<std::string, std::string>& values, const std::string& key) {
  const auto found = values.find(key);
  if (found == values.end() || found->second.size() < 2 || found->second.front() != '[' || found->second.back() != ']') {
    throw MapIoError("yaml 리스트가 아니다: " + key);
  }
  std::string inner = found->second.substr(1, found->second.size() - 2);
  std::vector<double> items;
  std::istringstream stream(inner);
  std::string item;
  while (std::getline(stream, item, ',')) {
    try {
      items.push_back(std::stod(trim(item)));
    } catch (...) {
      throw MapIoError("yaml 리스트 값이 숫자가 아니다: " + key);
    }
  }
  return items;
}

std::string read_file(const std::filesystem::path& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw MapIoError("파일을 읽을 수 없다: " + path.string());
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

struct Pgm {
  int width = 0;
  int height = 0;
  std::string pixels;
};

Pgm read_pgm(const std::filesystem::path& path) {
  const std::string data = read_file(path);
  std::vector<std::string> tokens;
  std::size_t index = 0;
  while (tokens.size() < 4) {
    while (index < data.size() && std::isspace(static_cast<unsigned char>(data[index])) != 0) {
      ++index;
    }
    if (index >= data.size()) {
      throw MapIoError("PGM 헤더가 모자란다: " + path.string());
    }
    if (data[index] == '#') {
      index = data.find('\n', index);
      if (index == std::string::npos) {
        throw MapIoError("PGM 주석이 끝나지 않는다: " + path.string());
      }
      ++index;
      continue;
    }
    const std::size_t start = index;
    while (index < data.size() && std::isspace(static_cast<unsigned char>(data[index])) == 0) {
      ++index;
    }
    tokens.push_back(data.substr(start, index - start));
  }
  if (tokens[0] != "P5" || tokens[3] != "255") {
    throw MapIoError("8비트 P5 PGM만 읽는다: " + path.string());
  }
  Pgm pgm;
  try {
    pgm.width = std::stoi(tokens[1]);
    pgm.height = std::stoi(tokens[2]);
  } catch (...) {
    throw MapIoError("PGM 크기가 숫자가 아니다: " + path.string());
  }
  const std::size_t pixels_start = index + 1;  // 최댓값 뒤 공백 한 칸
  if (pgm.width <= 0 || pgm.height <= 0 ||
      data.size() < pixels_start + static_cast<std::size_t>(pgm.width) * static_cast<std::size_t>(pgm.height)) {
    throw MapIoError("PGM 픽셀 수가 모자란다: " + path.string());
  }
  pgm.pixels = data.substr(pixels_start, static_cast<std::size_t>(pgm.width) * static_cast<std::size_t>(pgm.height));
  return pgm;
}

}  // namespace

msgs::OccupancyGrid load_map(const std::string& yaml_path, const std::string& frame_id, double stamp) {
  const std::filesystem::path yaml_file(yaml_path);
  const auto values = parse_yaml(read_file(yaml_file));
  const auto image_it = values.find("image");
  if (image_it == values.end()) {
    throw MapIoError("yaml에 image가 없다: " + yaml_path);
  }
  const std::string mode = values.count("mode") != 0 ? values.at("mode") : "trinary";
  if (mode != "trinary") {
    throw MapIoError("mode는 trinary만 지원한다: " + mode);
  }
  const double resolution = as_double(values, "resolution", 0.0);
  if (!(resolution > 0.0)) {
    throw MapIoError("resolution이 0 이하다: " + yaml_path);
  }
  const std::vector<double> origin = as_list(values, "origin");
  if (origin.size() != 3) {
    throw MapIoError("origin은 [x, y, yaw]여야 한다: " + yaml_path);
  }
  const int negate = static_cast<int>(as_double(values, "negate", 0.0));
  const double free_thresh = as_double(values, "free_thresh", kFreeThresh);
  const double occupied_thresh = as_double(values, "occupied_thresh", kOccupiedThresh);
  const Pgm pgm = read_pgm(yaml_file.parent_path() / image_it->second);

  msgs::OccupancyGrid grid;
  grid.header.stamp = stamp;
  grid.header.frame_id = frame_id;
  grid.info.map_load_time = stamp;
  grid.info.resolution = resolution;
  grid.info.width = static_cast<std::uint32_t>(pgm.width);
  grid.info.height = static_cast<std::uint32_t>(pgm.height);
  grid.info.origin.position = {origin[0], origin[1], 0.0};
  grid.info.origin.orientation = tfmath::from_yaw(origin[2]);
  grid.data.resize(static_cast<std::size_t>(pgm.width) * static_cast<std::size_t>(pgm.height), static_cast<std::int8_t>(-1));
  for (int y = 0; y < pgm.height; ++y) {
    for (int x = 0; x < pgm.width; ++x) {
      const int pixel = static_cast<unsigned char>(pgm.pixels[static_cast<std::size_t>((pgm.height - 1 - y) * pgm.width + x)]);
      const double occupied = negate != 0 ? static_cast<double>(pixel) / 255.0 : (255.0 - static_cast<double>(pixel)) / 255.0;
      grid.data[static_cast<std::size_t>(y) * pgm.width + x] =
          occupied > occupied_thresh ? static_cast<std::int8_t>(100)
                                     : (occupied < free_thresh ? static_cast<std::int8_t>(0) : static_cast<std::int8_t>(-1));
    }
  }
  return grid;
}

std::string save_map(const msgs::OccupancyGrid& grid, const std::string& path_without_extension, double free_thresh,
                     double occupied_thresh) {
  const auto& info = grid.info;
  if (info.width == 0 || info.height == 0 ||
      grid.data.size() != static_cast<std::size_t>(info.width) * static_cast<std::size_t>(info.height)) {
    throw MapIoError("지도 크기와 칸 수가 맞지 않는다");
  }
  const std::filesystem::path base(path_without_extension);
  if (!base.parent_path().empty()) {
    std::filesystem::create_directories(base.parent_path());
  }
  const std::filesystem::path image = base.string() + ".pgm";
  {
    std::ofstream stream(image, std::ios::binary);
    if (!stream) {
      throw MapIoError("PGM을 쓸 수 없다: " + image.string());
    }
    stream << "P5\n# CREATOR: krosc map_saver " << std::fixed << std::setprecision(3) << info.resolution
           << " m/pix\n" << info.width << " " << info.height << "\n255\n";
    for (int y = static_cast<int>(info.height) - 1; y >= 0; --y) {
      for (std::uint32_t x = 0; x < info.width; ++x) {
        const int value = grid.data[static_cast<std::size_t>(y) * info.width + x];
        const int pixel = value < 0 ? kUnknownPixel
                                    : (value >= occupied_thresh * 100.0 ? kWallPixel
                                                                       : (value <= free_thresh * 100.0 ? kFreePixel : kUnknownPixel));
        stream.put(static_cast<char>(pixel));
      }
    }
  }
  const std::filesystem::path yaml = base.string() + ".yaml";
  {
    std::ofstream stream(yaml);
    if (!stream) {
      throw MapIoError("yaml을 쓸 수 없다: " + yaml.string());
    }
    stream << "image: " << image.filename().string() << "\n"
           << "mode: trinary\n"
           << "resolution: " << std::setprecision(17) << info.resolution << "\n"
           << "origin: [" << info.origin.position.x << ", " << info.origin.position.y << ", "
           << tfmath::yaw_of(info.origin.orientation) << "]\n"
           << "negate: 0\n"
           << "occupied_thresh: " << occupied_thresh << "\n"
           << "free_thresh: " << free_thresh << "\n";
  }
  return yaml.string();
}

}  // namespace krosc::nodes
