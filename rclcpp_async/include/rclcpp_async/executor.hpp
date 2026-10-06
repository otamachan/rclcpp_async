// Copyright 2026 Tamaki Nishino
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

#include <coroutine>
#include <cstddef>
#include <functional>
#include <memory>
#include <stop_token>
#include <utility>

namespace rclcpp_async
{

inline constexpr size_t kDefaultStreamDepth = 10;

using StopCb = std::stop_callback<std::function<void()>>;

class Executor
{
public:
  virtual ~Executor() = default;

  // Post a callback to be executed on the executor thread (thread-safe).
  virtual void post(std::function<void()> fn) = 0;

  // Resume a coroutine directly. Call only from the executor thread.
  virtual void resume(std::coroutine_handle<> h) { h.resume(); }
};

template <typename DonePred, typename CancelAction>
inline void register_cancel(
  std::shared_ptr<StopCb> & out, std::stop_token token, Executor & ctx, std::coroutine_handle<> h,
  DonePred is_done, CancelAction action)
{
  out = std::make_shared<StopCb>(token, [&ctx, h, is_done, action, &out]() {
    ctx.post([&ctx, h, is_done, action, weak = std::weak_ptr(out)]() {
      // The awaiter owns `out`, and is_done / action may refer to it. Once it
      // is gone (resumed, or its frame destroyed after the stop was
      // requested) neither may be called.
      if (!weak.lock()) {
        return;
      }
      if (is_done()) {
        return;
      }
      action();
      ctx.resume(h);
    });
  });
}

// Resume h later on the executor thread, unless the awaiter owning `alive` is
// gone by then: its frame may be destroyed between the post and the resume.
inline void post_resume(Executor & ctx, std::coroutine_handle<> h, std::weak_ptr<void> alive)
{
  ctx.post([&ctx, h, alive = std::move(alive)]() {
    if (alive.lock()) {
      ctx.resume(h);
    }
  });
}

}  // namespace rclcpp_async
