// Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
// SPDX-License-Identifier: Apache-2.0
#ifndef PID_HPP
#define PID_HPP

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>

class PID
{
public:
    PID(
        rclcpp::Node* node,
        double kp,
        double kd,
        double ki,
        double min_output,
        double max_output,
        double integrator_min,
        double integrator_max,
        double dt,
        const std::string& axis_name
    );

    // Overload 1: usa el dt nominal almacenado en dt_
    // (compatibilidad con llamadas antiguas)
    double update(double current_value, double target_value);

    // Overload 2: usa el dt REAL medido por el wall timer del nodo —
    // PREFERIDO en el control loop. Si dt es inválido (<=0 o >1 s) cae al
    // nominal. Mismo patrón que SuperTwistingController::update.
    double update(double current_value, double target_value, double dt);

    void reset_integrator();

private:
    // Núcleo del cómputo — siempre llamado con un dt validado
    double compute(double current_value, double target_value, double dt);

    std::string axis_name_;
    double kp_, kd_, ki_;
    double min_output_, max_output_;
    double integrator_min_, integrator_max_;
    double dt_;

    double proportional_term_;
    double derivative_term_;
    double integral_term_;
    double prev_error_;
    double prev_value_;
    bool first_run_;
    rclcpp::Logger logger_;
    std::shared_ptr<rclcpp::Clock> clock_;   // para RCLCPP_WARN_THROTTLE
};

#endif // PID_HPP