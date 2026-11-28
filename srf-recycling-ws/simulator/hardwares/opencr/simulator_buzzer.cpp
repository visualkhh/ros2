#include "simulator_buzzer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <mutex>

#ifdef SRF_RECYCLING_WS_SIM_AUDIO
#include <QAudioFormat>
#include <QAudioSink>
#include <QCoreApplication>
#include <QIODevice>
#include <QMediaDevices>
#include <QMetaObject>
#endif

namespace srf_recycling_ws::sim {

namespace {

#ifdef SRF_RECYCLING_WS_SIM_AUDIO
constexpr int kSampleRate = 44100;
constexpr double kTwoPi = 6.283185307179586;
constexpr double kPi = 3.14159265358979323846;

struct Note {
  int frequency_hz = 0;
  int duration_ms = 0;
};

class ToneStream final : public QIODevice {
 public:
  explicit ToneStream(QObject* parent = nullptr) : QIODevice(parent) {
    open(QIODevice::ReadOnly);
  }

  void enqueue(int frequency_hz, int duration_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push_back(Note{frequency_hz, duration_ms});
    queue_.push_back(Note{0, 15});
  }

  qint64 readData(char* data, qint64 max_size) override {
    std::lock_guard<std::mutex> lock(mutex_);
    qint64 written = 0;
    while (written + 2 <= max_size) {
      if (remaining_ <= 0) {
        if (queue_.empty()) {
          std::fill(data + written, data + max_size, 0);
          written = max_size;
          break;
        }
        current_ = queue_.front();
        queue_.pop_front();
        remaining_ = current_.duration_ms * kSampleRate / 1000;
        phase_ = 0.0;
        continue;
      }
      // 피에조 부저처럼 사각파. 사인파보다 훨씬 잘 들린다.
      const double sample =
          current_.frequency_hz > 0 ? (phase_ < kPi ? 0.55 : -0.55) : 0.0;
      const auto value = static_cast<qint16>(sample * 32767.0);
      data[written++] = static_cast<char>(value & 0xFF);
      data[written++] = static_cast<char>((value >> 8) & 0xFF);
      phase_ += kTwoPi * static_cast<double>(current_.frequency_hz) / kSampleRate;
      if (phase_ >= kTwoPi) {
        phase_ -= kTwoPi;
      }
      --remaining_;
    }
    return written;
  }

  qint64 writeData(const char*, qint64) override { return 0; }
  qint64 bytesAvailable() const override {
    return QIODevice::bytesAvailable() + kSampleRate;
  }

 private:
  std::mutex mutex_;
  std::deque<Note> queue_;
  Note current_;
  int remaining_ = 0;
  double phase_ = 0.0;
};
#endif

}  // namespace

struct SimulatorBuzzer::Impl {
#ifdef SRF_RECYCLING_WS_SIM_AUDIO
  std::shared_ptr<ToneStream> stream;
  std::unique_ptr<QAudioSink> sink;
  bool available = false;

  Impl() {
    const QAudioDevice device = QMediaDevices::defaultAudioOutput();
    if (device.isNull()) {
      return;
    }
    QAudioFormat format;
    format.setSampleRate(kSampleRate);
    format.setChannelCount(1);
    format.setSampleFormat(QAudioFormat::Int16);
    if (!format.isValid()) {
      return;
    }
    stream = std::make_shared<ToneStream>();
    sink = std::make_unique<QAudioSink>(device, format);
    sink->setVolume(1.0F);
    sink->start(stream.get());
    available = sink->error() == QAudio::NoError;
  }

  ~Impl() {
    if (sink) {
      sink->stop();
    }
  }

  void enqueue(int frequency_hz, int duration_ms) {
    if (available && stream) {
      stream->enqueue(frequency_hz, duration_ms);
    }
  }
#else
  bool available = false;
  void enqueue(int, int) {}
#endif
};

SimulatorBuzzer::SimulatorBuzzer() : impl_(std::make_shared<Impl>()) {}

SimulatorBuzzer::~SimulatorBuzzer() = default;

void SimulatorBuzzer::play(int frequency_hz, int duration_ms) {
  if (!impl_ || !impl_->available) {
    return;
  }
  auto impl = impl_;
#ifdef SRF_RECYCLING_WS_SIM_AUDIO
  QObject* context = QCoreApplication::instance();
  if (context == nullptr) {
    impl->enqueue(frequency_hz, duration_ms);
    return;
  }
  QMetaObject::invokeMethod(
      context,
      [impl, frequency_hz, duration_ms] {
        impl->enqueue(frequency_hz, duration_ms);
      },
      Qt::QueuedConnection);
#else
  (void)frequency_hz;
  (void)duration_ms;
#endif
}

}  // namespace srf_recycling_ws::sim
