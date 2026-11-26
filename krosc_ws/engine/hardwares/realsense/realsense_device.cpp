#include "realsense_device.hpp"

#include <jpeglib.h>
#include <librealsense2/rs.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csetjmp>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace krosc_ws {

namespace {

using Clock = std::chrono::steady_clock;

double seconds_since(Clock::time_point time) {
  if (time.time_since_epoch().count() == 0) {
    return 0.0;
  }
  return std::chrono::duration<double>(Clock::now() - time).count();
}

struct JpegErrorManager {
  jpeg_error_mgr pub;
  std::jmp_buf jump;
};

void jpeg_error_exit(j_common_ptr info) {
  auto* error = reinterpret_cast<JpegErrorManager*>(info->err);
  std::longjmp(error->jump, 1);
}

std::vector<std::uint8_t> encode_jpeg(const std::uint8_t* bytes, int width, int height,
                                      int stride, int quality) {
  if (bytes == nullptr || width <= 0 || height <= 0 || stride < width * 3) {
    return {};
  }
  jpeg_compress_struct compressor{};
  JpegErrorManager error{};
  compressor.err = jpeg_std_error(&error.pub);
  error.pub.error_exit = jpeg_error_exit;
  unsigned char* output = nullptr;
  unsigned long output_size = 0;
  bool created = false;
  if (setjmp(error.jump) != 0) {
    if (created) {
      jpeg_destroy_compress(&compressor);
    }
    std::free(output);
    return {};
  }

  jpeg_create_compress(&compressor);
  created = true;
  jpeg_mem_dest(&compressor, &output, &output_size);
  compressor.image_width = static_cast<JDIMENSION>(width);
  compressor.image_height = static_cast<JDIMENSION>(height);
  compressor.input_components = 3;
  compressor.in_color_space = JCS_RGB;
  jpeg_set_defaults(&compressor);
  jpeg_set_quality(&compressor, std::clamp(quality, 1, 100), TRUE);
  jpeg_start_compress(&compressor, TRUE);
  while (compressor.next_scanline < compressor.image_height) {
    JSAMPROW row = const_cast<JSAMPROW>(
        bytes + static_cast<std::size_t>(compressor.next_scanline) * static_cast<std::size_t>(stride));
    jpeg_write_scanlines(&compressor, &row, 1);
  }
  jpeg_finish_compress(&compressor);
  std::vector<std::uint8_t> result(output, output + output_size);
  jpeg_destroy_compress(&compressor);
  std::free(output);
  return result;
}

std::string camera_info(const rs2::device& device, rs2_camera_info info) {
  try {
    return device.supports(info) ? std::string(device.get_info(info)) : std::string{};
  } catch (...) {
    return {};
  }
}

}  // namespace

struct RealSenseDevice::Impl {
  explicit Impl(Options value) : options(std::move(value)) {}

  Options options;
  rs2::pipeline pipeline;
  std::thread worker;
  std::atomic<bool> running{false};

  mutable std::mutex mutex;
  rs2::frame color_frame;
  rs2::frame depth_frame;
  std::uint64_t color_seq = 0;
  std::uint64_t depth_seq = 0;
  std::uint64_t color_taken = 0;
  std::uint64_t depth_taken = 0;
  std::uint64_t frames = 0;
  std::uint64_t color_frames = 0;
  std::uint64_t depth_frames = 0;
  Clock::time_point color_time{};
  Clock::time_point depth_time{};
  std::string serial;
  std::string firmware;
  std::string product;
};

RealSenseDevice::RealSenseDevice() : RealSenseDevice(Options{}) {}

RealSenseDevice::RealSenseDevice(Options options) : impl_(std::make_unique<Impl>(std::move(options))) {}

RealSenseDevice::~RealSenseDevice() { stop(); }

void RealSenseDevice::start() {
  if (impl_->running.load()) {
    return;
  }
  try {
    rs2::context context;
    const rs2::device_list devices = context.query_devices();
    if (devices.size() == 0) {
      std::fprintf(stderr, "[krosc_ws] RealSense: 장치 없음 — 카메라 없이 계속한다\n");
      std::fflush(stderr);
      return;
    }
    rs2::device device = devices[0];
    if (!impl_->options.serial.empty()) {
      bool found = false;
      for (std::size_t index = 0; index < devices.size(); ++index) {
        const rs2::device candidate = devices[index];
        if (camera_info(candidate, RS2_CAMERA_INFO_SERIAL_NUMBER) == impl_->options.serial) {
          device = candidate;
          found = true;
          break;
        }
      }
      if (!found) {
        std::fprintf(stderr, "[krosc_ws] RealSense: serial=%s 장치를 찾지 못했다\n",
                     impl_->options.serial.c_str());
        std::fflush(stderr);
        return;
      }
    }
    impl_->serial = camera_info(device, RS2_CAMERA_INFO_SERIAL_NUMBER);
    impl_->firmware = camera_info(device, RS2_CAMERA_INFO_FIRMWARE_VERSION);
    impl_->product = camera_info(device, RS2_CAMERA_INFO_NAME);

    rs2::config config;
    if (!impl_->serial.empty()) {
      config.enable_device(impl_->serial);
    }
    if (impl_->options.enable_color) {
      config.enable_stream(RS2_STREAM_COLOR, impl_->options.width, impl_->options.height,
                           RS2_FORMAT_RGB8, impl_->options.fps);
    }
    if (impl_->options.enable_depth) {
      config.enable_stream(RS2_STREAM_DEPTH, impl_->options.width, impl_->options.height,
                           RS2_FORMAT_Z16, impl_->options.fps);
    }
    impl_->pipeline.start(config);
    impl_->running.store(true);
    impl_->worker = std::thread([this] {
      try {
        while (impl_->running.load()) {
          const rs2::frameset frames = impl_->pipeline.wait_for_frames(1000);
          if (!frames) {
            continue;
          }
          bool received = false;
          if (impl_->options.enable_color) {
            if (const rs2::frame color = frames.get_color_frame()) {
              std::lock_guard<std::mutex> lock(impl_->mutex);
              impl_->color_frame = color;
              ++impl_->color_seq;
              ++impl_->color_frames;
              impl_->color_time = Clock::now();
              received = true;
            }
          }
          if (impl_->options.enable_depth) {
            if (const rs2::frame depth = frames.get_depth_frame()) {
              std::lock_guard<std::mutex> lock(impl_->mutex);
              impl_->depth_frame = depth;
              ++impl_->depth_seq;
              ++impl_->depth_frames;
              impl_->depth_time = Clock::now();
              received = true;
            }
          }
          if (received) {
            std::lock_guard<std::mutex> lock(impl_->mutex);
            ++impl_->frames;
          }
        }
      } catch (const rs2::error& error) {
        if (impl_->running.load()) {
          std::fprintf(stderr, "[krosc_ws] RealSense: %s\n", error.what());
          std::fflush(stderr);
        }
        impl_->running.store(false);
      }
    });

    std::printf("[krosc_ws] RealSense: %s serial=%s fw=%s color=%dx%d@%d depth=%dx%d@%d\n",
                impl_->product.empty() ? "D400" : impl_->product.c_str(),
                impl_->serial.c_str(), impl_->firmware.c_str(),
                impl_->options.width, impl_->options.height, impl_->options.fps,
                impl_->options.width, impl_->options.height, impl_->options.fps);
    std::fflush(stdout);
  } catch (const rs2::error& error) {
    std::fprintf(stderr, "[krosc_ws] RealSense 시작 실패: %s\n", error.what());
    std::fflush(stderr);
    impl_->running.store(false);
  } catch (const std::exception& error) {
    std::fprintf(stderr, "[krosc_ws] RealSense 시작 실패: %s\n", error.what());
    std::fflush(stderr);
    impl_->running.store(false);
  }
}

void RealSenseDevice::stop() {
  if (!impl_) {
    return;
  }
  impl_->running.store(false);
  if (impl_->worker.joinable()) {
    try {
      impl_->pipeline.stop();
    } catch (...) {
      // 이미 멈췄거나 장치가 뽑힌 경우.
    }
    impl_->worker.join();
  }
}

bool RealSenseDevice::connected() const {
  return impl_ && impl_->running.load();
}

std::optional<RealSenseImage> RealSenseDevice::take_color() {
  if (!impl_) {
    return std::nullopt;
  }
  rs2::frame stored;
  std::uint64_t sequence = 0;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->color_frame || impl_->color_seq == impl_->color_taken) {
      return std::nullopt;
    }
    stored = impl_->color_frame;
    sequence = impl_->color_seq;
  }
  impl_->color_taken = sequence;

  const rs2::video_frame frame(stored);
  const int width = frame.get_width();
  const int height = frame.get_height();
  const int stride = frame.get_stride_in_bytes();
  const auto format = frame.get_profile().as<rs2::video_stream_profile>().format();
  const auto* bytes = static_cast<const std::uint8_t*>(frame.get_data());
  std::vector<std::uint8_t> jpeg;
  if (format == RS2_FORMAT_RGB8) {
    jpeg = encode_jpeg(bytes, width, height, stride, impl_->options.jpeg_quality);
  } else if (format == RS2_FORMAT_BGR8) {
    std::vector<std::uint8_t> rgb(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3);
    for (int y = 0; y < height; ++y) {
      const auto* source = bytes + static_cast<std::size_t>(y) * static_cast<std::size_t>(stride);
      auto* target = rgb.data() + static_cast<std::size_t>(y) * static_cast<std::size_t>(width) * 3;
      for (int x = 0; x < width; ++x) {
        target[x * 3 + 0] = source[x * 3 + 2];
        target[x * 3 + 1] = source[x * 3 + 1];
        target[x * 3 + 2] = source[x * 3 + 0];
      }
    }
    jpeg = encode_jpeg(rgb.data(), width, height, width * 3, impl_->options.jpeg_quality);
  } else {
    return std::nullopt;
  }
  if (jpeg.empty()) {
    return std::nullopt;
  }
  RealSenseImage image;
  image.width = static_cast<std::uint32_t>(width);
  image.height = static_cast<std::uint32_t>(height);
  image.encoding = "jpeg";
  image.step = 0;
  image.data = std::move(jpeg);
  return image;
}

std::optional<RealSenseImage> RealSenseDevice::take_depth() {
  if (!impl_) {
    return std::nullopt;
  }
  rs2::frame stored;
  std::uint64_t sequence = 0;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->depth_frame || impl_->depth_seq == impl_->depth_taken) {
      return std::nullopt;
    }
    stored = impl_->depth_frame;
    sequence = impl_->depth_seq;
  }
  impl_->depth_taken = sequence;

  const rs2::depth_frame frame(stored);
  const auto format = frame.get_profile().as<rs2::video_stream_profile>().format();
  if (format != RS2_FORMAT_Z16) {
    return std::nullopt;
  }
  const int width = frame.get_width();
  const int height = frame.get_height();
  const int stride = frame.get_stride_in_bytes();
  const auto* bytes = static_cast<const std::uint8_t*>(frame.get_data());
  const std::size_t row_bytes = static_cast<std::size_t>(width) * 2;
  RealSenseImage image;
  image.width = static_cast<std::uint32_t>(width);
  image.height = static_cast<std::uint32_t>(height);
  image.encoding = "16UC1";
  image.step = static_cast<std::uint32_t>(row_bytes);
  image.data.resize(row_bytes * static_cast<std::size_t>(height));
  for (int y = 0; y < height; ++y) {
    std::memcpy(image.data.data() + static_cast<std::size_t>(y) * row_bytes,
                bytes + static_cast<std::size_t>(y) * static_cast<std::size_t>(stride),
                row_bytes);
  }
  return image;
}

RealSenseStats RealSenseDevice::stats() const {
  RealSenseStats result;
  if (!impl_) {
    return result;
  }
  std::lock_guard<std::mutex> lock(impl_->mutex);
  result.running = impl_->running.load();
  result.frames = impl_->frames;
  result.color_frames = impl_->color_frames;
  result.depth_frames = impl_->depth_frames;
  result.color_age = seconds_since(impl_->color_time);
  result.depth_age = seconds_since(impl_->depth_time);
  result.serial = impl_->serial;
  result.firmware = impl_->firmware;
  result.product = impl_->product;
  return result;
}

}  // namespace krosc_ws
