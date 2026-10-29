// Copyright 2026 LASER. Licensed under the Apache License, Version 2.0.
#ifndef LASER_GAZEBO_RESOURCES__MOTOR_EFFICIENCY_HPP_
#define LASER_GAZEBO_RESOURCES__MOTOR_EFFICIENCY_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace laser_gazebo_resources
{

// Simulation-thread state only. A new schedule freezes the current efficiency
// until its start time; the previous schedule is replaced, not queued.
class MotorEfficiency
{
public:
  struct Request
  {
    double start_time{0.0};  // Zero means the current simulation step.
    double target{1.0};
    uint8_t type{0};         // 0: abrupt, 1: linear.
    double duration{0.0};
  };

  static std::string validate(const Request & request)
  {
    if (!std::isfinite(request.target) || request.target < 0.0 || request.target > 1.0) {
      return "target_efficiency must be finite and within [0, 1]";
    }
    if (!std::isfinite(request.start_time) || request.start_time < 0.0) {
      return "failure_start_time must be finite and nonnegative";
    }
    if (request.type > 1) {
      return "failure_type must be ABRUPT (0) or LINEAR (1)";
    }
    if (!std::isfinite(request.duration) || request.duration < 0.0 ||
      (request.type == 1 && request.duration == 0.0))
    {
      return "transition_duration must be finite, nonnegative, and positive for LINEAR";
    }
    if (request.type == 0 && request.duration != 0.0) {
      return "ABRUPT requires transition_duration=0";
    }
    return {};
  }

  std::string schedule(const Request & request, double now)
  {
    const auto error = validate(request);
    if (!error.empty()) {
      return error;
    }
    if (request.start_time != 0.0 && request.start_time < now) {
      return "failure_start_time is already in the past at the simulation step";
    }
    const double start = request.start_time == 0.0 ? now : request.start_time;
    if (!std::isfinite(start + request.duration)) {
      return "transition end time overflows";
    }
    initial_ = value(now);
    request_ = request;
    request_.start_time = start;
    scheduled_ = true;
    return {};
  }

  double value(double now) const
  {
    if (!scheduled_ || now < request_.start_time) {
      return initial_;
    }
    if (request_.type == 0) {
      return request_.target;
    }
    const double fraction = std::clamp(
      (now - request_.start_time) / request_.duration, 0.0, 1.0);
    return std::clamp(initial_ + (request_.target - initial_) * fraction, 0.0, 1.0);
  }

  void reset(double initial = 1.0)
  {
    initial_ = initial;
    scheduled_ = false;
    request_ = Request{};
  }

  bool pending(double now) const {return scheduled_ && now < request_.start_time;}
  bool transitioning(double now) const
  {
    return scheduled_ && request_.type == 1 && now >= request_.start_time &&
           now < request_.start_time + request_.duration;
  }
  double start_time() const {return request_.start_time;}

private:
  double initial_{1.0};
  bool scheduled_{false};
  Request request_;
};

}  // namespace laser_gazebo_resources
#endif  // LASER_GAZEBO_RESOURCES__MOTOR_EFFICIENCY_HPP_
