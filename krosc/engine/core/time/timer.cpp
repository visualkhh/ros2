#include "krosc/time/timer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>
#include <utility>

namespace krosc {
namespace detail {

struct TimerState {
  double period = 0.0;
  std::function<void()> callback;
  std::shared_ptr<Strand> strand;
  std::string name;
  std::uint64_t id = 0;
  double next_due = 0.0;
  bool busy = false;
  std::atomic<bool> canceled{false};
  std::atomic<std::uint64_t> calls{0};
  std::atomic<std::uint64_t> missed{0};
};

struct HeapItem {
  double due = 0.0;
  std::uint64_t id = 0;
  bool operator>(const HeapItem& other) const {
    return due > other.due || (due == other.due && id > other.id);
  }
};

struct TimerManagerState {
  mutable std::mutex mutex;
  std::condition_variable cv;
  std::priority_queue<HeapItem, std::vector<HeapItem>, std::greater<>> heap;
  std::unordered_map<std::uint64_t, std::shared_ptr<TimerState>> timers;
  std::uint64_t next_id = 1;
  std::size_t inflight = 0;
  bool stopping = false;
  TimeSource* time_source = nullptr;
  Executor* executor = nullptr;
  std::thread scheduler;
  bool scheduler_started = false;
  bool scheduler_done = false;
};

namespace {

constexpr double kEpsilon = 1e-9;

void finish_task(const std::shared_ptr<TimerManagerState>& state, const std::shared_ptr<TimerState>& timer) {
  std::lock_guard<std::mutex> lock(state->mutex);
  timer->busy = false;
  if (state->inflight > 0) {
    --state->inflight;
  }
  state->cv.notify_all();
}

void run_timer(const std::shared_ptr<TimerManagerState>& state, const std::shared_ptr<TimerState>& timer) {
  if (timer->canceled.load(std::memory_order_acquire)) {
    finish_task(state, timer);
    return;
  }
  try {
    timer->callback();
    timer->calls.fetch_add(1, std::memory_order_relaxed);
  } catch (const std::exception& error) {
    std::fprintf(stderr, "[krosc] timer '%s' callback failed: %s\n", timer->name.c_str(), error.what());
  } catch (...) {
    std::fprintf(stderr, "[krosc] timer '%s' callback failed: unknown exception\n", timer->name.c_str());
  }
  finish_task(state, timer);
}

void fire_due(const std::shared_ptr<TimerManagerState>& state) {
  std::vector<std::shared_ptr<TimerState>> due;
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    if (state->stopping) {
      return;
    }
    const double now = state->time_source->monotonic();
    while (!state->heap.empty() && state->heap.top().due <= now + kEpsilon) {
      const HeapItem item = state->heap.top();
      state->heap.pop();
      auto it = state->timers.find(item.id);
      if (it == state->timers.end()) {
        continue;
      }
      const std::shared_ptr<TimerState>& timer = it->second;
      if (timer->canceled.load(std::memory_order_relaxed)) {
        state->timers.erase(it);
        continue;
      }
      if (std::abs(item.due - timer->next_due) > kEpsilon) {
        continue;  // 이미 새 일정으로 바뀐 낡은 힙 항목
      }
      int steps = 0;
      while (timer->next_due <= now + kEpsilon) {
        timer->next_due += timer->period;
        ++steps;
      }
      state->heap.push(HeapItem{timer->next_due, timer->id});
      if (timer->busy) {
        timer->missed.fetch_add(static_cast<std::uint64_t>(steps), std::memory_order_relaxed);
        continue;
      }
      if (steps > 1) {
        timer->missed.fetch_add(static_cast<std::uint64_t>(steps - 1), std::memory_order_relaxed);
      }
      timer->busy = true;
      ++state->inflight;
      due.push_back(timer);
    }
  }
  for (const auto& timer : due) {
    if (timer->strand) {
      timer->strand->post([state, timer] { run_timer(state, timer); });
    } else if (state->executor->inline_mode()) {
      run_timer(state, timer);
    } else {
      state->executor->post([state, timer] { run_timer(state, timer); });
    }
  }
}

void scheduler_loop(const std::shared_ptr<TimerManagerState>& state) {
  std::unique_lock<std::mutex> lock(state->mutex);
  while (!state->stopping) {
    state->scheduler_done = false;
    if (state->heap.empty()) {
      state->cv.wait(lock);
      continue;
    }
    const double now = state->time_source->monotonic();
    const double due = state->heap.top().due;
    if (due > now + kEpsilon) {
      state->cv.wait_for(lock, std::chrono::duration<double>(due - now));
      continue;
    }
    lock.unlock();
    fire_due(state);
    lock.lock();
  }
  state->scheduler_done = true;
  state->cv.notify_all();
}

}  // namespace
}  // namespace detail

double Timer::period() const { return state_ ? state_->period : 0.0; }
std::string Timer::name() const { return state_ ? state_->name : std::string{}; }
std::uint64_t Timer::calls() const { return state_ ? state_->calls.load(std::memory_order_relaxed) : 0; }
std::uint64_t Timer::missed() const { return state_ ? state_->missed.load(std::memory_order_relaxed) : 0; }
bool Timer::canceled() const { return state_ ? state_->canceled.load(std::memory_order_acquire) : true; }

void Timer::cancel() {
  if (!state_) {
    return;
  }
  state_->canceled.store(true, std::memory_order_release);
  if (auto manager = manager_.lock()) {
    std::lock_guard<std::mutex> lock(manager->mutex);
    manager->timers.erase(state_->id);
    manager->cv.notify_all();
  }
}

TimerManager::TimerManager(TimeSource& time_source, Executor& executor)
    : state_(std::make_shared<detail::TimerManagerState>()), time_source_(time_source), executor_(executor) {
  state_->time_source = &time_source_;
  state_->executor = &executor_;
  if (!time_source_.realtime()) {
    std::weak_ptr<detail::TimerManagerState> weak = state_;
    time_source_.add_listener([weak] {
      if (auto state = weak.lock()) {
        detail::fire_due(state);
      }
    });
  }
}

TimerManager::~TimerManager() { shutdown(); }

Timer TimerManager::create(double period, std::function<void()> callback, std::string name,
                           std::shared_ptr<Strand> strand) {
  if (!(period > 0.0)) {
    throw std::invalid_argument("타이머 주기는 0보다 커야 한다");
  }
  auto timer = std::make_shared<detail::TimerState>();
  timer->period = period;
  timer->callback = std::move(callback);
  timer->strand = std::move(strand);
  timer->name = std::move(name);
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->stopping) {
      throw std::runtime_error("타이머 매니저가 종료됐다");
    }
    timer->id = state_->next_id++;
    timer->next_due = time_source_.monotonic() + period;
    state_->timers.emplace(timer->id, timer);
    state_->heap.push(detail::HeapItem{timer->next_due, timer->id});
    if (time_source_.realtime() && !state_->scheduler_started) {
      state_->scheduler_started = true;
      std::weak_ptr<detail::TimerManagerState> weak = state_;
      state_->scheduler = std::thread([weak] {
        if (auto state = weak.lock()) {
          detail::scheduler_loop(state);
        }
      });
    }
  }
  state_->cv.notify_all();
  return Timer(state_, timer);
}

bool TimerManager::join(std::optional<double> timeout_seconds) {
  std::unique_lock<std::mutex> lock(state_->mutex);
  if (!timeout_seconds) {
    state_->cv.wait(lock, [this] { return state_->inflight == 0; });
    return true;
  }
  return state_->cv.wait_for(lock, std::chrono::duration<double>(*timeout_seconds),
                             [this] { return state_->inflight == 0; });
}

void TimerManager::shutdown() {
  std::thread thread;
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->stopping) {
      return;
    }
    state_->stopping = true;
    for (auto& [id, timer] : state_->timers) {
      (void)id;
      timer->canceled.store(true, std::memory_order_release);
    }
    state_->timers.clear();
    state_->heap = {};
    thread = std::move(state_->scheduler);
    state_->cv.notify_all();
  }
  if (thread.joinable() && thread.get_id() != std::this_thread::get_id()) {
    thread.join();
  }
  (void)join();
}

std::size_t TimerManager::timer_count() const {
  std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->timers.size();
}

std::vector<TimerManager::TimerStats> TimerManager::stats() const {
  std::lock_guard<std::mutex> lock(state_->mutex);
  std::vector<TimerStats> result;
  result.reserve(state_->timers.size());
  for (const auto& [id, timer] : state_->timers) {
    (void)id;
    result.push_back(TimerStats{timer->name.empty() ? "timer" : timer->name, timer->period,
                                timer->calls.load(std::memory_order_relaxed),
                                timer->missed.load(std::memory_order_relaxed)});
  }
  std::sort(result.begin(), result.end(), [](const TimerStats& a, const TimerStats& b) { return a.name < b.name; });
  return result;
}

}  // namespace krosc
