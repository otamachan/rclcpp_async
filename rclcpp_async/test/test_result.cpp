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

#include <gtest/gtest.h>

#include "rclcpp_async/result.hpp"

using namespace rclcpp_async;  // NOLINT(build/namespaces)

// ============================================================
// Result<T>
// ============================================================

TEST(ResultT, OkHoldsValue)
{
  auto r = Result<int>::Ok(42);
  EXPECT_TRUE(r.ok());
  EXPECT_FALSE(r.timeout());
  EXPECT_FALSE(r.cancelled());
  ASSERT_TRUE(r.value.has_value());
  EXPECT_EQ(*r.value, 42);
}

TEST(ResultT, TimeoutHasNoValue)
{
  auto r = Result<int>::Timeout();
  EXPECT_FALSE(r.ok());
  EXPECT_TRUE(r.timeout());
  EXPECT_FALSE(r.cancelled());
  EXPECT_FALSE(r.value.has_value());
}

TEST(ResultT, CancelledHasNoValue)
{
  auto r = Result<int>::Cancelled();
  EXPECT_FALSE(r.ok());
  EXPECT_FALSE(r.timeout());
  EXPECT_TRUE(r.cancelled());
  EXPECT_FALSE(r.value.has_value());
}

TEST(ResultT, ErrorHasMessage)
{
  auto r = Result<int>::Error("something went wrong");
  EXPECT_FALSE(r.ok());
  EXPECT_FALSE(r.timeout());
  EXPECT_FALSE(r.cancelled());
  EXPECT_EQ(r.error_msg, "something went wrong");
  EXPECT_FALSE(r.value.has_value());
}

TEST(ResultT, OkWithStringValue)
{
  auto r = Result<std::string>::Ok("hello");
  EXPECT_TRUE(r.ok());
  EXPECT_EQ(*r.value, "hello");
}

// ============================================================
// Result<void>
// ============================================================

TEST(ResultVoid, Ok)
{
  auto r = Result<void>::Ok();
  EXPECT_TRUE(r.ok());
  EXPECT_FALSE(r.timeout());
  EXPECT_FALSE(r.cancelled());
}

TEST(ResultVoid, Timeout)
{
  auto r = Result<void>::Timeout();
  EXPECT_FALSE(r.ok());
  EXPECT_TRUE(r.timeout());
}

TEST(ResultVoid, Cancelled)
{
  auto r = Result<void>::Cancelled();
  EXPECT_FALSE(r.ok());
  EXPECT_TRUE(r.cancelled());
}

TEST(ResultVoid, ErrorHasMessage)
{
  auto r = Result<void>::Error("fail");
  EXPECT_FALSE(r.ok());
  EXPECT_EQ(r.error_msg, "fail");
}
