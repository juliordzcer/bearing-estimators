// Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
// SPDX-License-Identifier: Apache-2.0
#ifndef UTILS_HPP
#define UTILS_HPP

#pragma once

#include <Eigen/Dense>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <map>
#include <string>
#include <vector>

namespace utils {

/**
 * @brief Converts a ROS Quaternion message to Euler angles (roll, pitch, yaw).
 * @param q The input geometry_msgs::msg::Quaternion.
 * @param roll The output roll angle in radians.
 * @param pitch The output pitch angle in radians.
 * @param yaw The output yaw angle in radians.
 */
inline void euler_from_quaternion(
    const geometry_msgs::msg::Quaternion &q,
    double &roll, double &pitch, double &yaw)
{
    tf2::Matrix3x3 m(tf2::Quaternion(q.x, q.y, q.z, q.w));
    m.getRPY(roll, pitch, yaw);
}

/**
 * @brief Converts Euler angles (roll, pitch, yaw) to a ROS Quaternion message.
 * @param roll The input roll angle in radians.
 * @param pitch The input pitch angle in radians.
 * @param yaw The input yaw angle in radians.
 * @return The corresponding geometry_msgs::msg::Quaternion.
 */
inline geometry_msgs::msg::Quaternion quaternion_from_euler(
    double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::Quaternion msg;
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();
    return msg;
}

/**
 * @brief Calculates the matrix of bearing errors in the GLOBAL frame.
 * @param robot_poses A map from robot names to their current positions.
 * @param robot_names A vector of all robot names (used for indexing).
 * @param adjacency_matrix A matrix defining which robot observes which.
 * @param gijA_d A map of desired bearings (observer -> target -> vector).
 * @return An Eigen::MatrixXd of size (N, N*3) containing the errors [e_x, e_y, e_z] for each pair.
 */
inline Eigen::MatrixXd calculate_bearing_errors(
    const std::map<std::string, Eigen::Vector3d>& robot_poses,
    const std::vector<std::string>& robot_names,
    const Eigen::MatrixXd& adjacency_matrix,
    const std::map<std::string, std::map<std::string, Eigen::Vector3d>>& gijA_d)
{
    const size_t n = robot_names.size();
    Eigen::MatrixXd bearing_errors(n, n * 3);
    bearing_errors.setZero();

    for (size_t i = 0; i < n; ++i) {
        const std::string& observer_name = robot_names[i];
        if (!robot_poses.count(observer_name)) continue;
        const Eigen::Vector3d& observer_pos = robot_poses.at(observer_name);

        for (size_t j = 0; j < n; ++j) {
            if (i == j) continue;
            
            if (adjacency_matrix(i, j) == 1) {
                const std::string& target_name = robot_names[j];
                if (!robot_poses.count(target_name)) continue;
                const Eigen::Vector3d& target_pos = robot_poses.at(target_name);

                const Eigen::Vector3d actual_diff_global = target_pos - observer_pos;
                Eigen::Vector3d actual_gij_global = Eigen::Vector3d::Zero();
                if (actual_diff_global.norm() > 1e-6) {
                    actual_gij_global = actual_diff_global.normalized();
                }

                Eigen::Vector3d desired_gij_global = Eigen::Vector3d::Zero();
                if (gijA_d.count(observer_name) && gijA_d.at(observer_name).count(target_name)) {
                    desired_gij_global = gijA_d.at(observer_name).at(target_name);
                }

                const Eigen::Vector3d error_vector = actual_gij_global - desired_gij_global;
                
                bearing_errors.block<1, 3>(i, j * 3) = error_vector.transpose();
            }
        }
    }
    return bearing_errors;
}

} // namespace utils

#endif // UTILS_HPP