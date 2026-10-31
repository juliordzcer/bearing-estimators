// Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
// SPDX-License-Identifier: Apache-2.0
#include "pid_bearing.hpp"
#include <algorithm>
#include <cmath>

// ---------------------------------------------------------------------------
// Constructors
// ---------------------------------------------------------------------------

PIDBearing::PIDBearing(
    std::shared_ptr<rclcpp::Node> node,
    double kp, double kd, double ki,
    double dt, double integral_limit,
    const std::string& axis_name)
    : PIDBearing(node.get(), kp, kd, ki, dt, integral_limit, axis_name)
{
}

PIDBearing::PIDBearing(
    rclcpp::Node* node,
    double kp, double kd, double ki,
    double dt, double integral_limit,
    const std::string& axis_name)
    : axis_name_(axis_name),
      kp_(kp), kd_(kd), ki_(ki),
      dt_(std::abs(dt)),
      integral_limit_(std::abs(integral_limit)),
      integral_term_(0.0),
      logger_(node->get_logger()),
      clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME))
{
    RCLCPP_INFO(logger_,
        "PIDBearing %s initialized: Kp=%.3f  Kd=%.3f  Ki=%.3f  "
        "dt=%.4f  integral_limit=%.3f",
        axis_name_.c_str(), kp_, kd_, ki_, dt_, integral_limit_);
}

// ---------------------------------------------------------------------------
// Public overloads
// ---------------------------------------------------------------------------

double PIDBearing::update(double current_value, double target_value,
                          double velocity_value, double velocity_target)
{
    return compute(current_value, target_value, velocity_value, velocity_target, dt_);
}

double PIDBearing::update(double current_value, double target_value,
                          double velocity_value, double velocity_target,
                          double dt)
{
    if (dt <= 0.0 || dt > 1.0) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
            "PIDBearing %s: invalid real_dt=%.6f, using nominal dt=%.6f",
            axis_name_.c_str(), dt, dt_);
        return compute(current_value, target_value, velocity_value, velocity_target, dt_);
    }
    return compute(current_value, target_value, velocity_value, velocity_target, dt);
}

// ---------------------------------------------------------------------------
// Core computation
// ---------------------------------------------------------------------------

double PIDBearing::compute(double current_value, double target_value,
                            double velocity_value, double velocity_target,
                            double dt)
{
    const double error     = current_value - target_value;      // gij_c - gij_d
    const double error_dot = velocity_value - velocity_target;  // dot_gij_c - 0

    const double proportional = kp_ * error;
    const double derivative   = kd_ * error_dot;

    if (std::abs(ki_) > 1e-9) {
        integral_term_ += ki_ * error * dt;
        integral_term_  = std::clamp(integral_term_, -integral_limit_, integral_limit_);
    }

    return proportional + integral_term_ + derivative;
}

// ---------------------------------------------------------------------------

void PIDBearing::reset_integrator()
{
    integral_term_ = 0.0;
    RCLCPP_DEBUG(logger_, "PIDBearing integrator for %s reset.", axis_name_.c_str());
}

void PIDBearing::set_gains(double kp, double kd, double ki)
{
    kp_ = kp; kd_ = kd; ki_ = ki;
    RCLCPP_INFO(logger_, "PIDBearing %s gains updated: Kp=%.3f  Kd=%.3f  Ki=%.3f",
                axis_name_.c_str(), kp_, kd_, ki_);
}