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
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>

#include "rclcpp_async/cancellation_token.hpp"
#include "rclcpp_async/result.hpp"

namespace rclcpp_async
{

class CoContext;

template <typename ServiceT>
struct SendRequestAwaiter
{
  using Response = typename ServiceT::Response::SharedPtr;

  CoContext & ctx;
  typename rclcpp::Client<ServiceT>::SharedPtr client;
  typename ServiceT::Request::SharedPtr request;
  CancellationToken * token = nullptr;
  Result<Response> result;

  void set_token(CancellationToken * t) { token = t; }

  bool await_ready()
  {
    if (token && token->is_cancelled()) {
      result = Result<Response>::Cancelled();
      return true;
    }
    return false;
  }

  // Defined inline because it's a template
  void await_suspend(std::coroutine_handle<> h);

  Result<Response> await_resume() { return std::move(result); }
};

}  // namespace rclcpp_async
