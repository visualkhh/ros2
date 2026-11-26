#include "krosc/sensor/sensor_manager.hpp"

#include <algorithm>
#include <utility>

namespace krosc::sensor {

SensorManager::~SensorManager() { stop(); }

bool SensorManager::add(std::string name, std::string type, std::unique_ptr<Sensor> sensor) {
  if (name.empty() || !sensor || find_entry(name) != nullptr) {
    return false;
  }
  entries_.push_back(Entry{std::move(name), std::move(type), std::move(sensor), false, false});
  return true;
}

bool SensorManager::start() {
  for (auto& entry : entries_) {
    if (entry.active) {
      continue;
    }
    if (!entry.initialized) {
      if (!entry.sensor->on_init(SensorInfo{entry.name, entry.type})) {
        stop();
        return false;
      }
      entry.initialized = true;
    }
    if (!entry.sensor->on_activate()) {
      stop();
      return false;
    }
    entry.active = true;
  }
  return true;
}

void SensorManager::stop() {
  // 켠 역순으로 내린다.
  for (auto it = entries_.rbegin(); it != entries_.rend(); ++it) {
    if (it->active) {
      it->sensor->on_deactivate();
      it->active = false;
    }
  }
}

std::size_t SensorManager::read(double time, double period) {
  std::size_t failures = 0;
  for (auto& entry : entries_) {
    if (!entry.active) {
      continue;
    }
    if (!entry.sensor->read(time, period)) {
      ++failures;
      ++read_failures_;
    }
  }
  return failures;
}

Sensor* SensorManager::find(const std::string& name) {
  Entry* entry = find_entry(name);
  return entry == nullptr ? nullptr : entry->sensor.get();
}

const Sensor* SensorManager::find(const std::string& name) const {
  const Entry* entry = find_entry(name);
  return entry == nullptr ? nullptr : entry->sensor.get();
}

std::size_t SensorManager::active_count() const {
  return static_cast<std::size_t>(
      std::count_if(entries_.begin(), entries_.end(), [](const Entry& entry) { return entry.active; }));
}

SensorManager::Entry* SensorManager::find_entry(const std::string& name) {
  const auto found = std::find_if(entries_.begin(), entries_.end(),
                                  [&name](const Entry& entry) { return entry.name == name; });
  return found == entries_.end() ? nullptr : &*found;
}

const SensorManager::Entry* SensorManager::find_entry(const std::string& name) const {
  const auto found = std::find_if(entries_.begin(), entries_.end(),
                                  [&name](const Entry& entry) { return entry.name == name; });
  return found == entries_.end() ? nullptr : &*found;
}

}  // namespace krosc::sensor
