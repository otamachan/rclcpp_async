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

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

namespace bench
{

using Clock = std::chrono::steady_clock;

struct Stats
{
  double mean_us;
  double min_us;
  double max_us;
};

inline Stats compute_stats(const std::vector<double> & samples_us)
{
  double sum = std::accumulate(samples_us.begin(), samples_us.end(), 0.0);
  double mean = sum / static_cast<double>(samples_us.size());
  double mn = *std::min_element(samples_us.begin(), samples_us.end());
  double mx = *std::max_element(samples_us.begin(), samples_us.end());
  return {mean, mn, mx};
}

inline void print_stats(const char * label, const Stats & s)
{
  std::cout << "  " << std::setw(12) << label << "  mean=" << std::setw(8) << s.mean_us
            << "  min=" << std::setw(8) << s.min_us << "  max=" << std::setw(8) << s.max_us << " us"
            << std::endl;
}

inline double to_us(Clock::time_point t0, Clock::time_point t1)
{
  return std::chrono::duration_cast<std::chrono::duration<double, std::micro>>(t1 - t0).count();
}

inline double to_us(std::chrono::nanoseconds ns)
{
  return std::chrono::duration_cast<std::chrono::duration<double, std::micro>>(ns).count();
}

}  // namespace bench
