// Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
// SPDX-License-Identifier: Apache-2.0
// pid.cpp - Enhanced Version

#include "pid.hpp"
#include <algorithm> 

PID::PID(
    rclcpp::Node* node,
    double kp,
    double kd,
    double ki,
    double min_output,
    double max_output,
    double integrator_min,
    double integrator_max,
    double dt,
    const std::string& axis_name)
    : axis_name_(axis_name),
      kp_(kp),
      kd_(kd),
      ki_(ki),
      min_output_(min_output),
      max_output_(max_output),
      integrator_min_(integrator_min),
      integrator_max_(integrator_max),
      dt_(dt),
      proportional_term_(0.0),
      derivative_term_(0.0),
      integral_term_(0.0),
      prev_error_(0.0),
      prev_value_(0.0),
      first_run_(true),
      logger_(node->get_logger()),
      clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME))
{
    // RCLCPP_INFO(logger_, "PID for %s initialized with Kp: %.2f, Kd: %.2f, Ki: %.2f",
    //             axis_name_.c_str(), kp_, kd_, ki_);
}

// Overload 1 — dt nominal (compatibilidad con llamadas antiguas)
double PID::update(double current_value, double target_value)
{
    return compute(current_value, target_value, dt_);
}

// Overload 2 — dt REAL medido por el wall timer del nodo
// Si el valor recibido es inválido, cae al dt nominal con un warning.
double PID::update(double current_value, double target_value, double dt)
{
    if (dt <= 0.0 || dt > 1.0) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
            "PID %s: invalid real_dt=%.6f, using nominal dt=%.6f",
            axis_name_.c_str(), dt, dt_);
        return compute(current_value, target_value, dt_);
    }
    return compute(current_value, target_value, dt);
}

double PID::compute(double current_value, double target_value, double dt)
{
    double error = target_value - current_value;

    proportional_term_ = kp_ * error;

    if (first_run_) {
        prev_error_ = error;
        prev_value_ = current_value; // Initialize prev_value_ as well for measurement derivative
        first_run_ = false;
    }

    // Using error derivative for derivative term (more common)
    derivative_term_ = kd_ * ((error - prev_error_) / dt);

    // Alternative for derivative term (less susceptible to setpoint changes):
    // derivative_term_ = -kd_ * ((current_value - prev_value_) / dt);

    integral_term_ += ki_ * error * dt;
    integral_term_ = std::clamp(integral_term_, integrator_min_, integrator_max_);

    double output = proportional_term_ + integral_term_ + derivative_term_;
    output = std::clamp(output, min_output_, max_output_);

    prev_error_ = error;
    prev_value_ = current_value; // Update prev_value_ for derivative calculation in next iteration

    return output;
}

void PID::reset_integrator()
{
    integral_term_ = 0.0;
    // When resetting integrator, it's often good practice to reset previous error/value for derivative too
    prev_error_ = 0.0; 
    // prev_value_ should ideally be reset to current_value if the PID is being "re-enabled"
    // but in this context, only resetting the integral is sufficient as the first_run_ handles it later.
    // RCLCPP_INFO(logger_, "PID integrator for %s reset.", axis_name_.c_str());
}