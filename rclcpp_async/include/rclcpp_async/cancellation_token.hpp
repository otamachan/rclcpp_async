// Copyright 2025 Tamaki Nishino
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

namespace rclcpp_async
{

class CancellationToken
{
  struct State
  {
    std::atomic<bool> cancelled{false};
    std::mutex mutex;
    std::vector<std::function<void()>> callbacks;
  };
  std::shared_ptr<State> state_ = std::make_shared<State>();

public:
  bool is_cancelled() const { return state_->cancelled.load(); }

  void cancel()
  {
    state_->cancelled.store(true);
    std::vector<std::function<void()>> cbs;
    {
      std::lock_guard lock(state_->mutex);
      cbs = std::move(state_->callbacks);
    }
    for (auto & cb : cbs) {
      cb();
    }
  }

  void on_cancel(std::function<void()> cb)
  {
    std::lock_guard lock(state_->mutex);
    if (state_->cancelled.load()) {
      cb();
    } else {
      state_->callbacks.push_back(std::move(cb));
    }
  }
};

}  // namespace rclcpp_async
