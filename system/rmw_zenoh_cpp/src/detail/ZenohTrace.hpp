#pragma once

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string_view>
#include <thread>
#include <unistd.h>

namespace rmw_zenoh_cpp::trace
{
inline bool pointcloud_topic(std::string_view topic)
{
  return topic.find("rslidar_points") != std::string_view::npos;
}

inline uint64_t steady_ns()
{
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
}

inline uint64_t tid()
{
  static thread_local const uint64_t id = std::hash<std::thread::id>{}(std::this_thread::get_id());
  return id;
}

inline FILE * file()
{
  static FILE * f = []() {
    const char * path = std::getenv("RMW_ZENOH_TRACE_FILE");
    if (!path || !*path) return static_cast<FILE *>(nullptr);
    std::string actual(path);
    const auto marker = actual.find("%p");
    if (marker != std::string::npos) actual.replace(marker, 2, std::to_string(static_cast<unsigned long>(getpid())));
    else actual += "." + std::to_string(static_cast<unsigned long>(getpid()));
    FILE * out = std::fopen(actual.c_str(), "a");
    if (out) std::setvbuf(out, nullptr, _IOFBF, 65536);
    return out;
  }();
  return f;
}

inline void event(const char * name, std::string_view topic = {}, uint64_t seq = 0,
  int64_t q_before = -1, int64_t q_after = -1, int64_t extra = -1)
{
  FILE * out = file();
  if (!out) return;
  static std::mutex m;
  std::lock_guard<std::mutex> lock(m);
  std::fprintf(out, "%s,%llu,%llu,%llu,%lld,%lld,%lld,%.*s\n", name,
    static_cast<unsigned long long>(steady_ns()),
    static_cast<unsigned long long>(tid()),
    static_cast<unsigned long long>(seq),
    static_cast<long long>(q_before), static_cast<long long>(q_after),
    static_cast<long long>(extra), static_cast<int>(topic.size()), topic.data());
}
}
