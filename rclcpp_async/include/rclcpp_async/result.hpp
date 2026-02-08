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

#include <optional>
#include <string>
#include <utility>

namespace rclcpp_async
{

template <typename T>
struct Result
{
  enum class Status
  {
    kOk,
    kTimeout,
    kCancelled,
    kError
  };

  Status status;
  std::optional<T> value;
  std::string error_msg;

  bool ok() const { return status == Status::kOk; }
  bool timeout() const { return status == Status::kTimeout; }
  bool cancelled() const { return status == Status::kCancelled; }

  static Result Ok(T v) { return {Status::kOk, std::move(v), {}}; }
  static Result Timeout() { return {Status::kTimeout, std::nullopt, {}}; }
  static Result Cancelled() { return {Status::kCancelled, std::nullopt, {}}; }
  static Result Error(std::string msg) { return {Status::kError, std::nullopt, std::move(msg)}; }
};

template <>
struct Result<void>
{
  enum class Status
  {
    kOk,
    kTimeout,
    kCancelled,
    kError
  };

  Status status;
  std::string error_msg;

  bool ok() const { return status == Status::kOk; }
  bool timeout() const { return status == Status::kTimeout; }
  bool cancelled() const { return status == Status::kCancelled; }

  static Result Ok() { return {Status::kOk, {}}; }
  static Result Timeout() { return {Status::kTimeout, {}}; }
  static Result Cancelled() { return {Status::kCancelled, {}}; }
  static Result Error(std::string msg) { return {Status::kError, std::move(msg)}; }
};

}  // namespace rclcpp_async
