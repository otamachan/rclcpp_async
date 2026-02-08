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

#include <memory>
#include <rclcpp_action/rclcpp_action.hpp>
#include <utility>

namespace rclcpp_async
{

template <typename ActionT>
class GoalContext
{
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  std::shared_ptr<GoalHandle> goal_handle_;
  std::shared_ptr<bool> aborted_;

public:
  GoalContext(std::shared_ptr<GoalHandle> gh, std::shared_ptr<bool> aborted)
  : goal_handle_(std::move(gh)), aborted_(std::move(aborted))
  {
  }

  const typename ActionT::Goal & goal() const { return *goal_handle_->get_goal(); }
  bool is_canceling() const { return goal_handle_->is_canceling(); }
  bool is_aborted() const { return *aborted_; }
  void abort() { *aborted_ = true; }

  void publish_feedback(const typename ActionT::Feedback & feedback)
  {
    goal_handle_->publish_feedback(std::make_shared<typename ActionT::Feedback>(feedback));
  }
};

}  // namespace rclcpp_async
