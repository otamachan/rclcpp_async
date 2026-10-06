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

#include <coroutine>
#include <functional>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <stop_token>
#include <utility>

#include "rclcpp_async/cancelled_exception.hpp"
#include "rclcpp_async/executor.hpp"

namespace rclcpp_async
{

class CoContext;

template <typename ServiceT>
struct SendRequestAwaiter
{
  using Response = typename ServiceT::Response::SharedPtr;

  // Mutable state is held via shared_ptr so that the response callback
  // registered on the rclcpp::Client can safely observe `done` even if
  // the awaiter (and its coroutine frame) has been destroyed after the
  // awaiting coroutine was cancelled.
  struct State
  {
    bool done = false;
    bool cancelled = false;
    Response response;
  };

  CoContext & ctx;
  typename rclcpp::Client<ServiceT>::SharedPtr client;
  typename ServiceT::Request::SharedPtr request;
  std::stop_token token;
  std::shared_ptr<StopCb> cancel_cb_;
  std::shared_ptr<State> state_;

  void set_token(std::stop_token t) { token = std::move(t); }

  bool await_ready()
  {
    if (token.stop_requested()) {
      state_->cancelled = true;
      return true;
    }
    return false;
  }

  // Defined inline because it's a template
  void await_suspend(std::coroutine_handle<> h);

  Response await_resume()
  {
    cancel_cb_.reset();
    if (state_->cancelled) {
      throw CancelledException{};
    }
    return std::move(state_->response);
  }
};

}  // namespace rclcpp_async
