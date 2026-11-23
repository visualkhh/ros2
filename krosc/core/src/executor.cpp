#include "krosc/executor.hpp"

#include <cstdio>
#include <exception>
#include <utility>

namespace krosc {

Executor::Executor(std::size_t threads) : threads_(threads) {
  if (threads_ > 0) {
    workers_.reserve(threads_);
    for (std::size_t i = 0; i < threads_; ++i) {
      workers_.emplace_back([this] { worker_loop(); });
    }
  }
}

Executor::~Executor() { shutdown(); }

void Executor::post(std::function<void()> task) {
  if (!task) {
    return;
  }
  if (threads_ == 0) {
    try {
      task();
    } catch (const std::exception& error) {
      std::fprintf(stderr, "[krosc] inline task failed: %s\n", error.what());
    } catch (...) {
      std::fprintf(stderr, "[krosc] inline task failed: unknown exception\n");
    }
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_) {
      return;
    }
    tasks_.push(std::move(task));
  }
  work_cv_.notify_one();
}

void Executor::join() {
  if (threads_ == 0) {
    return;
  }
  std::unique_lock<std::mutex> lock(mutex_);
  idle_cv_.wait(lock, [this] { return tasks_.empty() && running_ == 0; });
}

void Executor::shutdown() {
  if (threads_ == 0) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_) {
      return;  // 이미 종료 중: 아래 join은 소멸자에서 한 번만
    }
    stopping_ = true;
  }
  work_cv_.notify_all();
  for (auto& worker : workers_) {
    if (worker.joinable() && worker.get_id() != std::this_thread::get_id()) {
      worker.join();
    }
  }
  workers_.clear();
}

std::size_t Executor::pending() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return tasks_.size() + running_;
}

void Executor::worker_loop() {
  for (;;) {
    std::function<void()> task;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      work_cv_.wait(lock, [this] { return stopping_ || !tasks_.empty(); });
      if (tasks_.empty()) {
        if (stopping_) {
          return;
        }
        continue;
      }
      task = std::move(tasks_.front());
      tasks_.pop();
      ++running_;
    }
    try {
      task();
    } catch (const std::exception& error) {
      std::fprintf(stderr, "[krosc] executor task failed: %s\n", error.what());
    } catch (...) {
      std::fprintf(stderr, "[krosc] executor task failed: unknown exception\n");
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      --running_;
      if (tasks_.empty() && running_ == 0) {
        idle_cv_.notify_all();
      }
    }
  }
}

}  // namespace krosc
