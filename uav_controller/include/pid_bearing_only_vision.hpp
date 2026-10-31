// Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
// SPDX-License-Identifier: Apache-2.0
#ifndef PID_BEARING_ONLY_VISION_HPP
#define PID_BEARING_ONLY_VISION_HPP

// ROS 2 headers
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>

// Custom message for bearing measurements
#include <uav_msgs/msg/bearing_measurements_array.hpp>

// TF2 for quaternion conversions
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>

// Eigen for linear algebra
#include <Eigen/Dense>

// Standard library
#include <map>
#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <chrono>

#include "pid.hpp"
#include "pid_bearing.hpp"

class BearingOnlyVision : public rclcpp::Node
{
public:
    BearingOnlyVision();
    ~BearingOnlyVision();

    const std::vector<std::string>& get_robot_names() const { return robot_names_; }

private:
    // =========================================================================
    // ENUMS
    // =========================================================================
    enum class State
    {
        IDLE           = 0,
        AUTOMATIC      = 1,
        TAKING_OFF     = 2,
        LANDING        = 3,
        EMERGENCY_STOP = 4
    };

    // =========================================================================
    // CONSTANTS
    // =========================================================================
    static constexpr double POSE_TIMEOUT_SEC       = 0.5;
    static constexpr double VELOCITY_TIMEOUT_SEC   = 0.5;
    static constexpr double MIN_DISTANCE_THRESHOLD = 1e-6;

    // =========================================================================
    // INITIALIZATION METHODS
    // =========================================================================
    void declare_get_parameters();
    void initialize_robot_data();
    void publishers_and_subscribers();
    void initialize_pids();
    void initialize_pid_bearing_controllers();
    void reinitialize_pid_bearing_controllers();
    void setup_formation();
    void initialize_robot_colors();

    // =========================================================================
    // CALLBACKS
    // =========================================================================
    void state_changed_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg, const std::string& robot_name);
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg, const std::string& robot_name);
    void vision_bearings_callback(const uav_msgs::msg::BearingMeasurementsArray::SharedPtr msg);
    void vision_bearings_dot_callback(const uav_msgs::msg::BearingMeasurementsArray::SharedPtr msg);
    void parameters_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    // =========================================================================
    // CONTROL LOOP
    // =========================================================================
    void control_loop();

    // =========================================================================
    // BEARING & VISUALIZATION
    // =========================================================================
    Eigen::MatrixXd calculate_bearing_errors();
    void calculate_and_publish_bearing_errors();
    void calculate_and_publish_rms_bearing_error(const Eigen::MatrixXd& bearing_errors_matrix);
    void publish_visualization_markers();

    // =========================================================================
    // HELPER
    // =========================================================================
    bool is_leader(const std::string& name) const;

    // =========================================================================
    // NODE PARAMETERS
    // =========================================================================
    std::vector<std::string> robot_names_;
    std::vector<std::string> leader_names_;

    double control_loop_rate_;
    double takeoff_height_;
    double takeoff_height_tolerance_;
    double landing_threshold_;
    double landing_speed_factor_;
    double automatic_duration_sec_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    double bearing_loss_timeout_sec_;
    double trajectory_speed_;
    double formation_scale_;
    double amplitude_;
    double frequency_;
    double trajectory_time_scale_{1.0};

    // Bearing PID gains
    // k0: ganancia superficie deslizante  (s = e_dot + k0*e)
    // k1: ganancia termino proporcional
    // k2: ganancia termino integral
    double k0_x_, k1_x_, k2_x_;
    double k0_y_, k1_y_, k2_y_;
    double k0_z_, k1_z_, k2_z_;
    double pid_bearing_integral_limit_;

    // =========================================================================
    // ROBOT STATE DATA
    // =========================================================================
    std::map<std::string, Eigen::Vector3d> robot_orientations_;
    std::map<std::string, Eigen::Vector3d> robot_poses_;
    std::map<std::string, Eigen::Vector3d> robot_velocities_;
    std::map<std::string, rclcpp::Time>    last_pose_time_;
    std::map<std::string, rclcpp::Time>    last_velocity_time_;
    std::map<std::string, Eigen::Vector2d> initial_xy_takeoff_positions_;

    std::map<int, std::string>  robot_idx_to_name_;
    std::map<std::string, int>  robot_name_to_idx_;

    std::map<std::string, Eigen::Vector3d> last_valid_linear_commands_;
    std::map<std::string, rclcpp::Time>    last_valid_bearing_time_;
    std::map<std::string, std::map<std::string, rclcpp::Time>> bearing_stamps_;
    std::map<std::string, Eigen::Vector3d> follower_velocity_commands_;

    // =========================================================================
    // VISION BEARING DATA
    // =========================================================================
    /// Bearing medido en frame cámara: camera_bearings_[obs][tgt] = g_ij (unitario)
    std::map<std::string, std::map<std::string, Eigen::Vector3f>> camera_bearings_;

    /// Derivada del bearing medida en /vision_bearings (campo bearing_vector_dot del KF)
    std::map<std::string, std::map<std::string, Eigen::Vector3f>> camera_bearing_derivatives_;

    /// Derivada del bearing publicada en /vision_bearings_dot (bearing_vector = ḃ del KF)
    /// Se usa preferentemente en el controlador; si no hay dato se recurre a
    /// camera_bearing_derivatives_ como fallback.
    std::map<std::string, std::map<std::string, Eigen::Vector3f>> camera_bearing_dot_;

    Eigen::Matrix3d R_camera_to_robot_;

    // =========================================================================
    // CONTROLLERS
    // =========================================================================
    std::map<std::string, std::map<std::string, std::unique_ptr<PID>>>        pids_map_;
    std::map<std::string, std::map<std::string, std::unique_ptr<PIDBearing>>> pid_bearing_controllers_map_;

    // =========================================================================
    // PUBLISHERS & SUBSCRIBERS
    // =========================================================================
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> cmd_vel_pubs_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr>       enable_pubs_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr bearing_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr           rms_bearing_error_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr                             state_sub_;
    rclcpp::Subscription<uav_msgs::msg::BearingMeasurementsArray>::SharedPtr            vision_bearings_sub_;
    rclcpp::Subscription<uav_msgs::msg::BearingMeasurementsArray>::SharedPtr            vision_bearings_dot_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr                 parameters_sub_;
    std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr>  pose_subs_;
    std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> velocity_subs_;

    // =========================================================================
    // FORMATION DATA
    // =========================================================================
    std::map<std::string, Eigen::Vector3d>                        desired_positions_;
    Eigen::MatrixXd                                               adjacency_matrix_;
    Eigen::MatrixXd                                               laplacian_matrix_;
    std::map<std::string, std::map<std::string, Eigen::Vector3d>> gijA_d_;

    std::map<std::string, std_msgs::msg::ColorRGBA> robot_colors_;

    // =========================================================================
    // CONTROL STATE
    // =========================================================================
    State        current_state_;
    bool         enable_motors_;

    // Un bool de takeoff por robot — evita que drones que ya llegaron a la
    // altura objetivo interfieran con los que aun estan subiendo.
    std::map<std::string, bool> takeoff_complete_;

    rclcpp::Time automatic_start_time_;
    bool         automatic_start_time_initialized_;

    std::map<std::string, Eigen::Vector3d> leader_automatic_start_pos_;

    // Real dt — medido en cada llamada al wall timer
    rclcpp::Time  last_control_loop_time_;
    double        real_dt_;

    rclcpp::TimerBase::SharedPtr control_timer_;
};

#endif // PID_BEARING_ONLY_VISION_HPP