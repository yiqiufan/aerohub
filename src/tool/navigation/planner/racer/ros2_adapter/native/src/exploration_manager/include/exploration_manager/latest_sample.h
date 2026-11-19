#pragma once
#include <memory>
#include <mutex>
namespace fast_planner {
// The receiver never touches planner state; the FSM consumes one latest snapshot.
template<class T> class LatestSample {
 public:
  void receive(std::shared_ptr<const T> sample) {
    std::lock_guard<std::mutex> lock(mutex_); latest_ = std::move(sample);
  }
  std::shared_ptr<const T> take() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto sample = latest_; latest_.reset(); return sample;
  }
 private:
  std::mutex mutex_;
  std::shared_ptr<const T> latest_;
};
}
