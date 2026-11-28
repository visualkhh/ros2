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

namespace srf_recycling_ws {

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

// RealSense 워커 스레드에서만 호출한다. SensorManager가 take_*로 가져갈 때는
// 이미 decimation/JPEG encoding이 끝난 최신 이미지만 복사한다.
std::optional<RealSenseImage> make_color_image(const rs2::frame& stored, int decimation,
                                               int quality) {
  if (!stored) {
    return std::nullopt;
  }
  const rs2::video_frame frame(stored);
  const int width = frame.get_width();
  const int height = frame.get_height();
  const int stride = frame.get_stride_in_bytes();
  const auto format = frame.get_profile().as<rs2::video_stream_profile>().format();
  if (format != RS2_FORMAT_RGB8 && format != RS2_FORMAT_BGR8) {
    return std::nullopt;
  }
  const auto* bytes = static_cast<const std::uint8_t*>(frame.get_data());
  const int factor = std::max(1, decimation);
  const int output_width = width / factor;
  const int output_height = height / factor;
  if (output_width <= 0 || output_height <= 0) {
    return std::nullopt;
  }
  std::vector<std::uint8_t> rgb(static_cast<std::size_t>(output_width) *
                                static_cast<std::size_t>(output_height) * 3);
  const bool bgr = format == RS2_FORMAT_BGR8;
  for (int y = 0; y < output_height; ++y) {
    const auto* source =
        bytes + static_cast<std::size_t>(y * factor) * static_cast<std::size_t>(stride);
    auto* target =
        rgb.data() + static_cast<std::size_t>(y) * static_cast<std::size_t>(output_width) * 3;
    for (int x = 0; x < output_width; ++x) {
      const std::size_t source_index = static_cast<std::size_t>(x * factor) * 3;
      const std::size_t target_index = static_cast<std::size_t>(x) * 3;
      target[target_index + 0] = source[source_index + (bgr ? 2 : 0)];
      target[target_index + 1] = source[source_index + 1];
      target[target_index + 2] = source[source_index + (bgr ? 0 : 2)];
    }
  }
  auto jpeg = encode_jpeg(rgb.data(), output_width, output_height, output_width * 3, quality);
  if (jpeg.empty()) {
    return std::nullopt;
  }
  RealSenseImage image;
  image.width = static_cast<std::uint32_t>(output_width);
  image.height = static_cast<std::uint32_t>(output_height);
  image.encoding = "jpeg";
  image.step = 0;
  image.data = std::move(jpeg);
  return image;
}

std::optional<RealSenseImage> make_depth_image(const rs2::frame& stored, int decimation) {
  if (!stored) {
    return std::nullopt;
  }
  const rs2::depth_frame frame(stored);
  const auto format = frame.get_profile().as<rs2::video_stream_profile>().format();
  if (format != RS2_FORMAT_Z16) {
    return std::nullopt;
  }
  const int width = frame.get_width();
  const int height = frame.get_height();
  const int stride = frame.get_stride_in_bytes();
  const auto* bytes = static_cast<const std::uint8_t*>(frame.get_data());
  const int factor = std::max(1, decimation);
  const int output_width = width / factor;
  const int output_height = height / factor;
  if (output_width <= 0 || output_height <= 0) {
    return std::nullopt;
  }
  const std::size_t output_row_bytes = static_cast<std::size_t>(output_width) * 2;
  RealSenseImage image;
  image.width = static_cast<std::uint32_t>(output_width);
  image.height = static_cast<std::uint32_t>(output_height);
  image.encoding = "16UC1";
  image.step = static_cast<std::uint32_t>(output_row_bytes);
  image.data.resize(output_row_bytes * static_cast<std::size_t>(output_height));
  for (int y = 0; y < output_height; ++y) {
    const auto* source_row =
        bytes + static_cast<std::size_t>(y * factor) * static_cast<std::size_t>(stride);
    auto* target_row =
        image.data.data() + static_cast<std::size_t>(y) * output_row_bytes;
    for (int x = 0; x < output_width; ++x) {
      const std::size_t source = static_cast<std::size_t>(x * factor) * 2;
      target_row[static_cast<std::size_t>(x) * 2 + 0] = source_row[source + 0];
      target_row[static_cast<std::size_t>(x) * 2 + 1] = source_row[source + 1];
    }
  }
  return image;
}

}  // namespace

struct RealSenseDevice::Impl {
  explicit Impl(Options value) : options(std::move(value)) {}

  Options options;
  rs2::pipeline pipeline;
  std::thread worker;
  std::atomic<bool> running{false};

  mutable std::mutex mutex;
  RealSenseImage color_image;
  RealSenseImage depth_image;
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
  // 워커가 예외로 끝나면 running=false지만 std::thread는 joinable로 남는다.
  // 이 상태에서 새 스레드를 대입하면 std::terminate라서 먼저 정리한다.
  if (impl_->worker.joinable()) {
    impl_->worker.join();
    try {
      impl_->pipeline.stop();
    } catch (...) {
      // 이미 멈췄거나 장치가 뽑힌 경우.
    }
  }
  try {
    rs2::context context;
    const rs2::device_list devices = context.query_devices();
    if (devices.size() == 0) {
      std::fprintf(stderr, "[srf_recycling_ws] RealSense: 장치 없음 — 카메라 없이 계속한다\n");
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
        std::fprintf(stderr, "[srf_recycling_ws] RealSense: serial=%s 장치를 찾지 못했다\n",
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
    {
      std::lock_guard<std::mutex> lock(impl_->mutex);
      impl_->color_image = {};
      impl_->depth_image = {};
      impl_->color_seq = 0;
      impl_->depth_seq = 0;
      impl_->color_taken = 0;
      impl_->depth_taken = 0;
    }
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
              bool encode = false;
              {
                std::lock_guard<std::mutex> lock(impl_->mutex);
                ++impl_->color_frames;
                impl_->color_time = Clock::now();
                // 아직 센서가 이전 프레임을 가져가지 않았으면 인코딩을 건너뛴다.
                // 인코딩/decimation은 소비자(SensorManager)가 아니라 이 워커가 한다.
                encode = impl_->color_seq == impl_->color_taken;
              }
              if (encode) {
                if (auto image = make_color_image(color, impl_->options.color_decimation,
                                                  impl_->options.jpeg_quality)) {
                  std::lock_guard<std::mutex> lock(impl_->mutex);
                  impl_->color_image = std::move(*image);
                  ++impl_->color_seq;
                }
              }
              received = true;
            }
          }
          if (impl_->options.enable_depth) {
            if (const rs2::frame depth = frames.get_depth_frame()) {
              bool encode = false;
              {
                std::lock_guard<std::mutex> lock(impl_->mutex);
                ++impl_->depth_frames;
                impl_->depth_time = Clock::now();
                encode = impl_->depth_seq == impl_->depth_taken;
              }
              if (encode) {
                if (auto image = make_depth_image(depth, impl_->options.depth_decimation)) {
                  std::lock_guard<std::mutex> lock(impl_->mutex);
                  impl_->depth_image = std::move(*image);
                  ++impl_->depth_seq;
                }
              }
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
          std::fprintf(stderr, "[srf_recycling_ws] RealSense: %s\n", error.what());
          std::fflush(stderr);
        }
        impl_->running.store(false);
      }
    });

    std::printf(
        "[srf_recycling_ws] RealSense: %s serial=%s fw=%s stream color=%dx%d@%d depth=%dx%d@%d "
        "-> publish color=%dx%d depth=%dx%d\n",
        impl_->product.empty() ? "D400" : impl_->product.c_str(), impl_->serial.c_str(),
        impl_->firmware.c_str(), impl_->options.width, impl_->options.height,
        impl_->options.fps, impl_->options.width, impl_->options.height, impl_->options.fps,
        impl_->options.width / std::max(1, impl_->options.color_decimation),
        impl_->options.height / std::max(1, impl_->options.color_decimation),
        impl_->options.width / std::max(1, impl_->options.depth_decimation),
        impl_->options.height / std::max(1, impl_->options.depth_decimation));
    std::fflush(stdout);
  } catch (const rs2::error& error) {
    std::fprintf(stderr, "[srf_recycling_ws] RealSense 시작 실패: %s\n", error.what());
    std::fflush(stderr);
    impl_->running.store(false);
  } catch (const std::exception& error) {
    std::fprintf(stderr, "[srf_recycling_ws] RealSense 시작 실패: %s\n", error.what());
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
  // 연결이 끊긴 뒤 늦게 take_*가 불려도 옛 프레임을 내보내지 않는다.
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->color_image = {};
  impl_->depth_image = {};
}

void RealSenseDevice::connect() {
  if (!is_connected()) {
    start();
  }
}

void RealSenseDevice::disconnect() {
  if (is_connected()) {
    stop();
  }
}

bool RealSenseDevice::is_connected() const {
  return impl_ && impl_->running.load();
}

std::optional<RealSenseImage> RealSenseDevice::take_color() {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->color_image.data.empty() || impl_->color_seq == impl_->color_taken) {
    return std::nullopt;
  }
  impl_->color_taken = impl_->color_seq;
  return impl_->color_image;
}

std::optional<RealSenseImage> RealSenseDevice::take_depth() {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->depth_image.data.empty() || impl_->depth_seq == impl_->depth_taken) {
    return std::nullopt;
  }
  impl_->depth_taken = impl_->depth_seq;
  return impl_->depth_image;
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

}  // namespace srf_recycling_ws
