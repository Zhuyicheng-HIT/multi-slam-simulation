
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <numeric>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include <gz/msgs/image.pb.h>
#include <gz/msgs/world_stats.pb.h>
#include <gz/transport/Node.hh>

namespace
{
using Clock = std::chrono::steady_clock;

int64_t stamp_ns(const gz::msgs::Image &_message)
{
  return _message.header().stamp().sec() * 1000000000LL +
         _message.header().stamp().nsec();
}

double percentile(std::vector<double> _values, double _q)
{
  if (_values.empty()) return 0.0;
  std::sort(_values.begin(), _values.end());
  const double position = _q * static_cast<double>(_values.size() - 1);
  const std::size_t low = static_cast<std::size_t>(std::floor(position));
  const std::size_t high = static_cast<std::size_t>(std::ceil(position));
  const double fraction = position - static_cast<double>(low);
  return _values[low] * (1.0 - fraction) + _values[high] * fraction;
}

double mean(const std::vector<double> &_values)
{
  return _values.empty() ? 0.0 :
    std::accumulate(_values.begin(), _values.end(), 0.0) /
    static_cast<double>(_values.size());
}

struct StreamStats
{
  uint64_t count{0};
  Clock::time_point first{};
  Clock::time_point last{};
  std::vector<double> intervals_ms;

  void update()
  {
    const auto now = Clock::now();
    if (count == 0)
      first = now;
    else
      intervals_ms.push_back(std::chrono::duration<double, std::milli>(now - last).count());
    last = now;
    ++count;
  }

  double overall_hz() const
  {
    if (count < 2) return 0.0;
    return static_cast<double>(count - 1) /
      std::chrono::duration<double>(last - first).count();
  }

  std::vector<double> rates() const
  {
    std::vector<double> result;
    result.reserve(intervals_ms.size());
    for (double interval : intervals_ms)
      if (interval > 0.0) result.push_back(1000.0 / interval);
    return result;
  }
};

class Probe
{
public:
  Probe(const std::string &_color, const std::string &_depth, const std::string &_stats)
  {
    if (!node_.Subscribe(_color, &Probe::color_callback, this) ||
        !node_.Subscribe(_depth, &Probe::depth_callback, this))
      throw std::runtime_error("could not subscribe to RGB-D Gazebo topics");
    if (!_stats.empty())
      node_.Subscribe(_stats, &Probe::stats_callback, this);
  }

  void color_callback(const gz::msgs::Image &_message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    color_.update();
    match(stamp_ns(_message), true);
  }

  void depth_callback(const gz::msgs::Image &_message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    depth_.update();
    match(stamp_ns(_message), false);
  }

  void stats_callback(const gz::msgs::WorldStatistics &_message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!_message.paused()) rtf_.push_back(_message.real_time_factor());
  }

  void match(int64_t _stamp, bool _color)
  {
    auto &own = _color ? color_stamps_ : depth_stamps_;
    auto &other = _color ? depth_stamps_ : color_stamps_;
    auto &own_order = _color ? color_order_ : depth_order_;
    auto &other_order = _color ? depth_order_ : color_order_;
    const auto found = other.find(_stamp);
    if (found != other.end())
    {
      other.erase(found);
      const auto order_found = std::find(other_order.begin(), other_order.end(), _stamp);
      if (order_found != other_order.end()) other_order.erase(order_found);
      ++exact_pairs_;
    }
    else
    {
      own.insert(_stamp);
      own_order.push_back(_stamp);
      while (own_order.size() > 16)
      {
        unmatched_drops_ += own.erase(own_order.front());
        own_order.pop_front();
      }
    }
  }

  void output(const std::string &_csv)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto color_rates = color_.rates();
    const auto depth_rates = depth_.rates();
    const double color_longest = color_.intervals_ms.empty() ? 0.0 :
      *std::max_element(color_.intervals_ms.begin(), color_.intervals_ms.end());
    const double depth_longest = depth_.intervals_ms.empty() ? 0.0 :
      *std::max_element(depth_.intervals_ms.begin(), depth_.intervals_ms.end());
    std::cout << std::fixed << std::setprecision(4)
              << "color_count=" << color_.count << '\n'
              << "depth_count=" << depth_.count << '\n'
              << "exact_pair_count=" << exact_pairs_ << '\n'
              << "unmatched_drop_count=" << unmatched_drops_ << '\n'
              << "color_mean_hz=" << color_.overall_hz() << '\n'
              << "color_median_hz=" << percentile(color_rates, 0.5) << '\n'
              << "color_p05_hz=" << percentile(color_rates, 0.05) << '\n'
              << "color_p95_hz=" << percentile(color_rates, 0.95) << '\n'
              << "color_min_hz=" << (color_rates.empty() ? 0.0 : *std::min_element(color_rates.begin(), color_rates.end())) << '\n'
              << "color_max_hz=" << (color_rates.empty() ? 0.0 : *std::max_element(color_rates.begin(), color_rates.end())) << '\n'
              << "color_longest_interval_ms=" << color_longest << '\n'
              << "depth_mean_hz=" << depth_.overall_hz() << '\n'
              << "depth_median_hz=" << percentile(depth_rates, 0.5) << '\n'
              << "depth_p05_hz=" << percentile(depth_rates, 0.05) << '\n'
              << "depth_p95_hz=" << percentile(depth_rates, 0.95) << '\n'
              << "depth_min_hz=" << (depth_rates.empty() ? 0.0 : *std::min_element(depth_rates.begin(), depth_rates.end())) << '\n'
              << "depth_max_hz=" << (depth_rates.empty() ? 0.0 : *std::max_element(depth_rates.begin(), depth_rates.end())) << '\n'
              << "depth_longest_interval_ms=" << depth_longest << '\n'
              << "rtf_mean=" << mean(rtf_) << '\n'
              << "rtf_median=" << percentile(rtf_, 0.5) << '\n';
    if (!_csv.empty())
    {
      std::ofstream file(_csv);
      file << "color_count,depth_count,exact_pair_count,unmatched_drop_count,color_mean_hz,"
              "color_median_hz,color_p05_hz,color_p95_hz,color_min_hz,color_max_hz,"
              "color_longest_interval_ms,depth_mean_hz,depth_median_hz,depth_p05_hz,"
              "depth_p95_hz,depth_min_hz,depth_max_hz,depth_longest_interval_ms,rtf_mean,rtf_median\n";
      file << color_.count << ',' << depth_.count << ',' << exact_pairs_ << ','
           << unmatched_drops_ << ',' << color_.overall_hz() << ','
           << percentile(color_rates, 0.5) << ',' << percentile(color_rates, 0.05) << ','
           << percentile(color_rates, 0.95) << ','
           << (color_rates.empty() ? 0.0 : *std::min_element(color_rates.begin(), color_rates.end())) << ','
           << (color_rates.empty() ? 0.0 : *std::max_element(color_rates.begin(), color_rates.end())) << ','
           << color_longest << ',' << depth_.overall_hz() << ','
           << percentile(depth_rates, 0.5) << ',' << percentile(depth_rates, 0.05) << ','
           << percentile(depth_rates, 0.95) << ','
           << (depth_rates.empty() ? 0.0 : *std::min_element(depth_rates.begin(), depth_rates.end())) << ','
           << (depth_rates.empty() ? 0.0 : *std::max_element(depth_rates.begin(), depth_rates.end())) << ','
           << depth_longest << ',' << mean(rtf_) << ',' << percentile(rtf_, 0.5) << '\n';
    }
  }

private:
  gz::transport::Node node_;
  std::mutex mutex_;
  StreamStats color_;
  StreamStats depth_;
  std::vector<double> rtf_;
  std::unordered_set<int64_t> color_stamps_;
  std::unordered_set<int64_t> depth_stamps_;
  std::deque<int64_t> color_order_;
  std::deque<int64_t> depth_order_;
  uint64_t exact_pairs_{0};
  uint64_t unmatched_drops_{0};
};
}  // namespace

int main(int argc, char **argv)
{
  std::string prefix = "/front/d435i/gz";
  std::string stats_topic;
  std::string csv_path;
  double duration = 30.0;
  for (int i = 1; i < argc; ++i)
  {
    const std::string argument(argv[i]);
    if (argument == "--prefix" && i + 1 < argc) prefix = argv[++i];
    else if (argument == "--stats-topic" && i + 1 < argc) stats_topic = argv[++i];
    else if (argument == "--csv" && i + 1 < argc) csv_path = argv[++i];
    else if (argument == "--duration" && i + 1 < argc) duration = std::stod(argv[++i]);
    else
    {
      std::cerr << "usage: gz_rgbd_rate_probe [--prefix TOPIC_PREFIX] "
                   "[--stats-topic TOPIC] [--duration SECONDS] [--csv PATH]\n";
      return 2;
    }
  }
  try
  {
    Probe probe(prefix + "/image", prefix + "/depth_image", stats_topic);
    std::this_thread::sleep_for(std::chrono::duration<double>(duration));
    probe.output(csv_path);
  }
  catch (const std::exception &error)
  {
    std::cerr << "probe failed: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
