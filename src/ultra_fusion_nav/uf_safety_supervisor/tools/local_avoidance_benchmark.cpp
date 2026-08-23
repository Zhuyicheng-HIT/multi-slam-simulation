#include "uf_safety_supervisor/local_avoidance_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using uf_safety_supervisor::ConservativeLocalPlanner;

namespace
{
std::vector<Eigen::Vector3d> wall(double x, double low, double high)
{
  std::vector<Eigen::Vector3d> result;
  for (double y = low; y <= high; y += 0.15) {result.emplace_back(x, y, 0.0);}
  return result;
}

double percentile(std::vector<double> values, const double p)
{
  std::sort(values.begin(), values.end());
  return values[static_cast<std::size_t>(p * static_cast<double>(values.size() - 1U))];
}
}

int main()
{
  ConservativeLocalPlanner planner;
  const std::vector<std::pair<std::string, std::vector<Eigen::Vector3d>>> scenarios{
    {"wall", wall(3.0, -1.5, 1.5)},
    {"column", wall(3.0, -0.2, 0.2)},
    {"sudden", wall(2.0, -1.0, 1.0)},
    {"reblocked", wall(2.7, -1.8, 0.5)}};
  std::vector<double> latency;
  std::size_t successes = 0U;
  std::size_t collisions = 0U;
  double minimum_clearance = 1.0e9;
  for (int repeat = 0; repeat < 100; ++repeat) {
    for (const auto & scenario : scenarios) {
      const auto started = std::chrono::steady_clock::now();
      const auto result = planner.plan({0.0, 0.0, 0.0}, {6.0, 0.0, 0.0}, scenario.second);
      latency.push_back(std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count());
      successes += result.success ? 1U : 0U;
      collisions += result.success && !result.verified ? 1U : 0U;
      if (result.success) {minimum_clearance = std::min(minimum_clearance, result.minimum_clearance_m);}
    }
  }
  std::cout << std::fixed << std::setprecision(3)
            << "plans=" << latency.size()
            << " successes=" << successes
            << " collisions=" << collisions
            << " latency_p50_ms=" << percentile(latency, 0.50)
            << " latency_p95_ms=" << percentile(latency, 0.95)
            << " latency_p99_ms=" << percentile(latency, 0.99)
            << " minimum_clearance_m=" << minimum_clearance << '\n';
  return successes == latency.size() && collisions == 0U ? 0 : 1;
}
