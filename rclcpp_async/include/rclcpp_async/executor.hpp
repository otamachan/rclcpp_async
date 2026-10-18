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

#include <atomic>
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

// A suspended coroutine, owned by its awaiter. Whoever resumes it later holds
// a weak_ptr, which expires with the awaiter. The handle is taken once, so of
// the completion and the cancellation only the first resumes it.
class Waiting
{
  std::atomic<std::coroutine_handle<>> h_;

public:
  explicit Waiting(std::coroutine_handle<> h) : h_(h) {}
  std::coroutine_handle<> take() { return h_.exchange(nullptr); }
};

// Take `w` and resume it here. False when its awaiter is gone or already taken.
inline bool resume_now(Executor & ctx, const std::weak_ptr<Waiting> & w)
{
  auto s = w.lock();
  auto h = s ? s->take() : nullptr;
  if (!h) {
    return false;
  }
  ctx.resume(h);
  return true;
}

// Take `w` now, run `claimed`, and resume it on the executor unless its awaiter
// is gone by then.
template <typename Claimed>
inline void post_resume(Executor & ctx, std::weak_ptr<Waiting> w, Claimed claimed)
{
  auto s = w.lock();
  auto h = s ? s->take() : nullptr;
  if (!h) {
    return;
  }
  claimed();
  ctx.post([w = std::move(w), h]() {
    if (w.lock()) {
      h.resume();
    }
  });
}

inline void post_resume(Executor & ctx, std::weak_ptr<Waiting> w)
{
  post_resume(ctx, std::move(w), []() {});
}

// On a stop request, resume `waiting` on the executor after `action`, unless
// it was taken or its awaiter destroyed first. `action` may use the awaiter.
template <typename CancelAction>
inline void register_cancel(
  std::shared_ptr<StopCb> & out, std::stop_token token, Executor & ctx,
  const std::shared_ptr<Waiting> & waiting, CancelAction action)
{
  out = std::make_shared<StopCb>(token, [&ctx, w = std::weak_ptr(waiting), action]() {
    ctx.post([&ctx, w, action]() {
      auto s = w.lock();
      auto h = s ? s->take() : nullptr;
      if (!h) {
        return;
      }
      action();
      ctx.resume(h);
    });
  });
}

}  // namespace rclcpp_async
