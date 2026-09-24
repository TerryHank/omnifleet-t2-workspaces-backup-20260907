#pragma once
#include <mutex>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <limits>

// Shared continuous clock mapping for Airy points and IMU. Never replace
// acquisition times with arrival times. A slow frequency servo compensates
// crystal drift; the minimum residual in a window rejects scheduling delay.
class SensorClockBridge
{
 public:
  double convert(double device, double host)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(device) || !std::isfinite(host))
      throw std::runtime_error("Invalid Airy clock sample");
    if (!initialized_)
    {
      anchor_device_ = latest_ = window_start_ = device;
      anchor_host_ = host;
      initialized_ = true;
    }
    if (device < latest_ - 1.0)
      throw std::runtime_error("Airy acquisition clock reset: restart the sensor driver");
    const double mapped = anchor_host_ + (device-anchor_device_)*rate_;
    // Interleaved IMU/point callbacks may arrive slightly out of order. Do not
    // clamp them to the last callback: that would corrupt acquisition order.
    if (device >= latest_)
    {
      minimum_residual_ = std::min(minimum_residual_, host-mapped);
      if (device-window_start_ >= 1.0)
      {
        // Reanchor at the existing value: changing rate cannot jump time.
        // 200 ppm is a correction-rate bound, not a freshness tolerance.
        rate_ = 1.0 + std::clamp(minimum_residual_/20.0, -0.0002, 0.0002);
        anchor_device_ = device;
        anchor_host_ = mapped;
        window_start_ = device;
        minimum_residual_ = std::numeric_limits<double>::infinity();
      }
      latest_ = device;
    }
    return mapped;
  }
 private:
  std::mutex mutex_;
  bool initialized_ = false;
  double anchor_device_ = 0, anchor_host_ = 0, latest_ = 0;
  double window_start_ = 0, rate_ = 1;
  double minimum_residual_ = std::numeric_limits<double>::infinity();
};
