// Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
// SPDX-License-Identifier: Apache-2.0
#ifndef PIDBEARING_HPP
#define PIDBEARING_HPP

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>

class PIDBearing
{
public:
    // Constructor with shared_ptr (delegates to raw pointer constructor)
    PIDBearing(
        std::shared_ptr<rclcpp::Node> node,
        double kp, double kd, double ki,
        double dt,
        double integral_limit,
        const std::string& axis_name);

    // Primary constructor
    PIDBearing(
        rclcpp::Node* node,
        double kp, double kd, double ki,
        double dt,
        double integral_limit,
        const std::string& axis_name);

    // Uses stored nominal dt_
    double update(double current_value, double target_value,
                  double velocity_value, double velocity_target);

    // Uses externally measured real dt — preferred in the control loop
    double update(double current_value, double target_value,
                  double velocity_value, double velocity_target,
                  double dt);

    void reset_integrator();
    void set_gains(double kp, double kd, double ki);

private:
    double compute(double current_value, double target_value,
                   double velocity_value, double velocity_target,
                   double dt);

    std::string axis_name_;
    double kp_, kd_, ki_;
    double dt_;              // nominal / fallback dt
    double integral_limit_;

    double integral_term_;
    rclcpp::Logger                    logger_;
    std::shared_ptr<rclcpp::Clock>    clock_;   // for RCLCPP_WARN_THROTTLE
};

#endif // PIDBEARING_HPP