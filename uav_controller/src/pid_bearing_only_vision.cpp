// Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
// SPDX-License-Identifier: Apache-2.0
#include "pid_bearing_only_vision.hpp"

#include <cmath>
#include <algorithm>
#include <sstream>

// =============================================================================
// CONSTRUCTOR / DESTRUCTOR
// =============================================================================

BearingOnlyVision::BearingOnlyVision()
    : Node("crazyflie_pid_bearing_vision_node"),
      current_state_(State::IDLE),
      enable_motors_(false),
      automatic_start_time_(rclcpp::Time(0, 0, get_clock()->get_clock_type())),
      automatic_start_time_initialized_(false),
      last_control_loop_time_(rclcpp::Time(0, 0, get_clock()->get_clock_type())),
      real_dt_(0.0)
{
    // Fixed leader set – referenced throughout the class
    leader_names_ = {"crazyflie1", "crazyflie7", "crazyflie4"};

    // Camera-to-robot rotation (identity: camera frame == robot frame)
    R_camera_to_robot_ << 1, 0, 0,
                          0, 1, 0,
                          0, 0, 1;

    declare_get_parameters();
    initialize_robot_data();
    publishers_and_subscribers();
    initialize_pids();
    initialize_pid_bearing_controllers();
    setup_formation();
    initialize_robot_colors();

    // create_timer (not create_wall_timer) uses the node clock, so with
    // use_sim_time it ticks on simulated time instead of wall time.
    control_timer_ = this->create_timer(
        std::chrono::duration<double>(1.0 / control_loop_rate_),
        std::bind(&BearingOnlyVision::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Bearing Only Vision Controller initialized.");
}

BearingOnlyVision::~BearingOnlyVision()
{
    RCLCPP_INFO(this->get_logger(), "Bearing Only Vision Controller shutting down.");
}

// =============================================================================
// HELPERS
// =============================================================================

bool BearingOnlyVision::is_leader(const std::string& name) const
{
    for (const auto& ln : leader_names_) {
        if (name == ln) return true;
    }
    return false;
}

/**
 * @brief Normaliza un ángulo al rango (-pi, pi].
 */
static double wrap_angle(double angle)
{
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

/**
 * @brief Devuelve el target de yaw deseado para un robot dado.
 */
static double get_yaw_target(const std::string& robot_name)
{
    if (robot_name == "crazyflie2" || robot_name == "crazyflie3")
        return M_PI;
    return 0.0;
}

// =============================================================================
// PARAMETERS
// =============================================================================

void BearingOnlyVision::declare_get_parameters()
{
    this->declare_parameter("initial_conditions_vec", std::vector<double>({}));

    this->declare_parameter("robot_names",               std::vector<std::string>({"crazyflie1", "crazyflie2", "crazyflie3"}));
    this->declare_parameter("control_loop_rate",         30.0);
    this->declare_parameter("takeoff_height",            1.5);
    this->declare_parameter("takeoff_height_tolerance",  0.1);
    this->declare_parameter("landing_threshold",         0.05);
    this->declare_parameter("landing_speed_factor",      0.7);
    this->declare_parameter("automatic_duration_sec",    100.0);
    this->declare_parameter("max_linear_velocity",       1.0);
    this->declare_parameter("max_angular_velocity",      1.0);
    this->declare_parameter("trajectory_speed",          0.1);
    this->declare_parameter("formation_scale",           1.0);
    this->declare_parameter("amplitude",                 0.1);
    this->declare_parameter("frequency",                 0.04);
    this->declare_parameter("trajectory_time_scale",     1.0);
    this->declare_parameter("bearing_loss_timeout_sec",  2.0);

    // PID X
    this->declare_parameter("pid.kp_x",             0.5);
    this->declare_parameter("pid.ki_x",             0.0);
    this->declare_parameter("pid.kd_x",             0.0);
    this->declare_parameter("pid.min_output_x",    -1.0);
    this->declare_parameter("pid.max_output_x",     1.0);
    this->declare_parameter("pid.integrator_min_x",-0.5);
    this->declare_parameter("pid.integrator_max_x", 0.5);

    // PID Y
    this->declare_parameter("pid.kp_y",             0.5);
    this->declare_parameter("pid.ki_y",             0.0);
    this->declare_parameter("pid.kd_y",             0.0);
    this->declare_parameter("pid.min_output_y",    -1.0);
    this->declare_parameter("pid.max_output_y",     1.0);
    this->declare_parameter("pid.integrator_min_y",-0.5);
    this->declare_parameter("pid.integrator_max_y", 0.5);

    // PID Z
    this->declare_parameter("pid.kp_z",             0.7);
    this->declare_parameter("pid.ki_z",             0.01);
    this->declare_parameter("pid.kd_z",             0.1);
    this->declare_parameter("pid.min_output_z",    -0.5);
    this->declare_parameter("pid.max_output_z",     1.0);
    this->declare_parameter("pid.integrator_min_z",-0.5);
    this->declare_parameter("pid.integrator_max_z", 0.5);

    // PID Yaw
    this->declare_parameter("pid.kp_yaw",             0.5);
    this->declare_parameter("pid.ki_yaw",             0.0);
    this->declare_parameter("pid.kd_yaw",             0.0);
    this->declare_parameter("pid.min_output_yaw",    -1.0);
    this->declare_parameter("pid.max_output_yaw",     1.0);
    this->declare_parameter("pid.integrator_min_yaw",-0.5);
    this->declare_parameter("pid.integrator_max_yaw", 0.5);

    // Bearing PID gains (per axis)
    this->declare_parameter("pid_bearing.k0_x", 0.0);
    this->declare_parameter("pid_bearing.k1_x", 0.0);
    this->declare_parameter("pid_bearing.k2_x", 0.0);
    this->declare_parameter("pid_bearing.k0_y", 0.0);
    this->declare_parameter("pid_bearing.k1_y", 0.0);
    this->declare_parameter("pid_bearing.k2_y", 0.0);
    this->declare_parameter("pid_bearing.k0_z", 0.0);
    this->declare_parameter("pid_bearing.k1_z", 0.0);
    this->declare_parameter("pid_bearing.k2_z", 0.0);
    this->declare_parameter("pid_bearing.integral_limit", 0.1);

    // --- Retrieve values ---
    this->get_parameter("robot_names",              robot_names_);
    this->get_parameter("control_loop_rate",        control_loop_rate_);
    this->get_parameter("takeoff_height",           takeoff_height_);
    this->get_parameter("takeoff_height_tolerance", takeoff_height_tolerance_);
    this->get_parameter("landing_threshold",        landing_threshold_);
    this->get_parameter("landing_speed_factor",     landing_speed_factor_);
    this->get_parameter("automatic_duration_sec",   automatic_duration_sec_);
    this->get_parameter("max_linear_velocity",      max_linear_velocity_);
    this->get_parameter("max_angular_velocity",     max_angular_velocity_);
    this->get_parameter("trajectory_speed",         trajectory_speed_);
    this->get_parameter("formation_scale",          formation_scale_);
    this->get_parameter("amplitude",                amplitude_);
    this->get_parameter("frequency",                frequency_);
    this->get_parameter("trajectory_time_scale",    trajectory_time_scale_);
    this->get_parameter("bearing_loss_timeout_sec", bearing_loss_timeout_sec_);

    this->get_parameter("pid_bearing.k0_x", k0_x_);
    this->get_parameter("pid_bearing.k1_x", k1_x_);
    this->get_parameter("pid_bearing.k2_x", k2_x_);
    this->get_parameter("pid_bearing.k0_y", k0_y_);
    this->get_parameter("pid_bearing.k1_y", k1_y_);
    this->get_parameter("pid_bearing.k2_y", k2_y_);
    this->get_parameter("pid_bearing.k0_z", k0_z_);
    this->get_parameter("pid_bearing.k1_z", k1_z_);
    this->get_parameter("pid_bearing.k2_z", k2_z_);
    this->get_parameter("pid_bearing.integral_limit", pid_bearing_integral_limit_);
}

// =============================================================================
// INITIALIZATION
// =============================================================================

void BearingOnlyVision::initialize_robot_data()
{
    std::vector<double> flat_ic;
    this->get_parameter("initial_conditions_vec", flat_ic);

    const bool use_params = !flat_ic.empty() && (flat_ic.size() % 4 == 0);

    if (use_params) {
        RCLCPP_INFO(this->get_logger(), "Using dynamic initial conditions from launch file.");
    } else {
        RCLCPP_WARN(this->get_logger(), "No dynamic initial conditions found. Using defaults.");
    }

    const std::vector<std::array<double, 4>> default_data = {{
        { 0.450,  0.200, 0.00, 0.00},
        { 0.450, -0.200, 0.00, 0.00},
        { 0.000,  0.000, 0.00, 0.00},
    }};

    const rclcpp::Time zero_time(0, 0, get_clock()->get_clock_type());

    for (size_t i = 0; i < robot_names_.size(); ++i)
    {
        const auto& name = robot_names_[i];

        robot_orientations_[name]   = Eigen::Vector3d::Zero();
        robot_poses_[name]          = Eigen::Vector3d::Zero();
        robot_velocities_[name]     = Eigen::Vector3d::Zero();
        last_pose_time_[name]       = this->now();
        last_velocity_time_[name]   = this->now();
        robot_idx_to_name_[i]       = name;
        robot_name_to_idx_[name]    = static_cast<int>(i);

        last_valid_linear_commands_[name] = Eigen::Vector3d::Zero();
        last_valid_bearing_time_[name]    = zero_time;
        follower_velocity_commands_[name] = Eigen::Vector3d::Zero();

        takeoff_complete_[name] = false;

        double init_x = 0.0, init_y = 0.0;
        if (use_params) {
            const size_t base = i * 4;
            if (base + 1 < flat_ic.size()) {
                init_x = flat_ic[base];
                init_y = flat_ic[base + 1];
            }
        } else if (i < default_data.size()) {
            init_x = default_data[i][0];
            init_y = default_data[i][1];
        }

        initial_xy_takeoff_positions_[name] = Eigen::Vector2d(init_x, init_y);
        RCLCPP_INFO(this->get_logger(), "Robot %s initial pos -> X: %.3f, Y: %.3f",
                    name.c_str(), init_x, init_y);
    }
}

void BearingOnlyVision::publishers_and_subscribers()
{
    auto reliable_qos = rclcpp::QoS(10)
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    auto latched_qos = rclcpp::QoS(10)
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    // BEST_EFFORT depth=1: pose/vel llegan a >50 Hz desde Gazebo bridge.
    // Con RELIABLE depth=15 se acumula cola durante picos de CPU y se
    // reproduce en burst, causando saltos de control. Solo se necesita el
    // dato más reciente.
    auto sensor_qos = rclcpp::QoS(2)
        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // RELIABLE depth=1 para cmd_vel: el ros_gz_bridge suscribe con RELIABLE
    // (publisher BEST_EFFORT sería incompatible). depth=1 evita acumular
    // comandos viejos en cola sin sacrificar compatibilidad con el bridge.
    auto cmd_vel_qos = rclcpp::QoS(1)
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    for (const auto& name : robot_names_) {
        cmd_vel_pubs_[name] = this->create_publisher<geometry_msgs::msg::Twist>(
            "/" + name + "/cmd_vel", cmd_vel_qos);

        enable_pubs_[name] = this->create_publisher<std_msgs::msg::Bool>(
            "/" + name + "/enable", latched_qos);

        pose_subs_[name] = this->create_subscription<geometry_msgs::msg::Pose>(
            "/" + name + "/pose", sensor_qos,
            [this, name](const geometry_msgs::msg::Pose::SharedPtr msg) {
                this->pose_callback(msg, name);
            });

        velocity_subs_[name] = this->create_subscription<geometry_msgs::msg::Twist>(
            "/" + name + "/velocity", sensor_qos,
            [this, name](const geometry_msgs::msg::Twist::SharedPtr msg) {
                this->velocity_callback(msg, name);
            });
    }

    bearing_error_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/bearing_errors", reliable_qos);

    rms_bearing_error_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/rms_bearing_error", reliable_qos);

    visualization_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/swarm_visualization_markers", reliable_qos);

    state_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/state", latched_qos,
        std::bind(&BearingOnlyVision::state_changed_callback, this, std::placeholders::_1));

    vision_bearings_sub_ = this->create_subscription<uav_msgs::msg::BearingMeasurementsArray>(
        "/vision_bearings", rclcpp::SensorDataQoS(),
        std::bind(&BearingOnlyVision::vision_bearings_callback, this, std::placeholders::_1));

    // ── NUEVO: subscriber al topic dedicado de bearing_dot ───────────────────
    vision_bearings_dot_sub_ = this->create_subscription<uav_msgs::msg::BearingMeasurementsArray>(
        "/vision_bearings_dot", rclcpp::SensorDataQoS(),
        std::bind(&BearingOnlyVision::vision_bearings_dot_callback, this, std::placeholders::_1));

    parameters_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/controller_parameters", 10,
        std::bind(&BearingOnlyVision::parameters_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Publishers and subscribers initialized.");
}

void BearingOnlyVision::initialize_robot_colors()
{
    const std::set<std::string> leaders = {"crazyflie1", "crazyflie4", "crazyflie7"};

    for (const auto& name : robot_names_) {
        std_msgs::msg::ColorRGBA color;
        if (leaders.count(name)) {
            color.r = 1.0f; color.g = 0.8f; color.b = 0.0f; color.a = 1.0f;
        } else {
            color.r = 0.0f; color.g = 0.85f; color.b = 0.85f; color.a = 1.0f;
        }
        robot_colors_[name] = color;
    }
}

void BearingOnlyVision::initialize_pids()
{
    const double dt = 1.0 / control_loop_rate_;

    auto get_pid_params = [&](const std::string& axis)
        -> std::tuple<double, double, double, double, double, double, double>
    {
        double kp, ki, kd, min_out, max_out, int_min, int_max;
        this->get_parameter("pid.kp_"             + axis, kp);
        this->get_parameter("pid.ki_"             + axis, ki);
        this->get_parameter("pid.kd_"             + axis, kd);
        this->get_parameter("pid.min_output_"     + axis, min_out);
        this->get_parameter("pid.max_output_"     + axis, max_out);
        this->get_parameter("pid.integrator_min_" + axis, int_min);
        this->get_parameter("pid.integrator_max_" + axis, int_max);
        return {kp, ki, kd, min_out, max_out, int_min, int_max};
    };

    const std::vector<std::string> axes = {"x", "y", "z", "yaw"};

    for (const auto& robot_name : robot_names_) {
        for (const auto& axis : axes) {
            auto [kp, ki, kd, min_out, max_out, int_min, int_max] = get_pid_params(axis);
            pids_map_[robot_name][axis] = std::make_unique<PID>(
                this, kp, ki, kd, min_out, max_out, int_min, int_max, dt,
                robot_name + "_" + axis);
        }
    }
    RCLCPP_INFO(this->get_logger(), "PID controllers initialized for all robots and axes.");
}

void BearingOnlyVision::initialize_pid_bearing_controllers()
{
    const double dt = 1.0 / control_loop_rate_;

    for (const auto& robot_name : robot_names_) {
        if (is_leader(robot_name)) continue;

        pid_bearing_controllers_map_[robot_name]["x"] = std::make_unique<PIDBearing>(
            this, k0_x_, k1_x_, k2_x_, dt, pid_bearing_integral_limit_,
            robot_name + "_pid_bearing_x");
        pid_bearing_controllers_map_[robot_name]["y"] = std::make_unique<PIDBearing>(
            this, k0_y_, k1_y_, k2_y_, dt, pid_bearing_integral_limit_,
            robot_name + "_pid_bearing_y");
        pid_bearing_controllers_map_[robot_name]["z"] = std::make_unique<PIDBearing>(
            this, k0_z_, k1_z_, k2_z_, dt, pid_bearing_integral_limit_,
            robot_name + "_pid_bearing_z");
    }
    RCLCPP_INFO(this->get_logger(), "PID bearing controllers initialized for followers.");
}

void BearingOnlyVision::reinitialize_pid_bearing_controllers()
{
    pid_bearing_controllers_map_.clear();
    initialize_pid_bearing_controllers();
    RCLCPP_INFO(this->get_logger(), "Bearing PID controllers reinitialized with updated gains.");
}

void BearingOnlyVision::setup_formation()
{
    const std::vector<Eigen::Vector3d> base_positions = {
        {   0.3200,   0.3200, 0.0000},
        {   0.6664,   0.1600, 0.0000},
        {   0.6664,  -0.1600, 0.0000},
        {   0.3200,  -0.3200, 0.0000},
        {  -0.0264,  -0.1600, 0.0000},
        {  -0.0264,   0.1600, 0.0000},
        {   0.3200,   0.0000, 0.0000},
    };

    if (robot_names_.size() != base_positions.size()) {
        RCLCPP_ERROR(this->get_logger(),
            "Mismatch: %zu robot names vs %zu base positions. Formation setup aborted.",
            robot_names_.size(), base_positions.size());
        return;
    }

    for (size_t i = 0; i < robot_names_.size(); ++i) {
        desired_positions_[robot_names_[i]] = base_positions[i] * formation_scale_;
    }

    const size_t n = robot_names_.size();
    adjacency_matrix_.resize(n, n);
    laplacian_matrix_.resize(n, n);
    adjacency_matrix_.setZero();

    std::vector<int> cf_idx(11, -1);
    for (size_t i = 0; i < n; ++i) {
        const std::string& name = robot_names_[i];
        if (name.rfind("crazyflie", 0) == 0) {
            try {
                int num = std::stoi(name.substr(9));
                if (num >= 1 && num <= 10) cf_idx[num] = static_cast<int>(i);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(),
                    "Cannot parse robot index from name '%s': %s", name.c_str(), e.what());
            }
        }
    }

    auto connect = [&](int from, int to) {
        if (cf_idx[from] != -1 && cf_idx[to] != -1) {
            adjacency_matrix_(cf_idx[from], cf_idx[to]) = 1;
        } else {
            RCLCPP_WARN(this->get_logger(),
                "Cannot connect crazyflie%d -> crazyflie%d: robot(s) not found.", from, to);
        }
    };

    connect(2, 1);
    connect(2, 7);
    connect(3, 4);
    connect(3, 7);
    connect(6, 1);
    connect(6, 7);
    connect(5, 4);
    connect(5, 7);

    Eigen::MatrixXd degree = Eigen::MatrixXd::Zero(n, n);
    for (size_t i = 0; i < n; ++i) {
        degree(i, i) = adjacency_matrix_.row(i).sum();
    }
    laplacian_matrix_ = degree - adjacency_matrix_;

    for (const auto& [obs_name, obs_pos] : desired_positions_) {
        for (const auto& [tgt_name, tgt_pos] : desired_positions_) {
            if (obs_name == tgt_name) continue;
            const Eigen::Vector3d diff = tgt_pos - obs_pos;
            const double norm = diff.norm();
            gijA_d_[obs_name][tgt_name] = (norm > MIN_DISTANCE_THRESHOLD)
                                           ? Eigen::Vector3d(diff / norm)
                                           : Eigen::Vector3d::Zero();
        }
    }

    RCLCPP_INFO(this->get_logger(), "Formation configuration complete.");
}

// =============================================================================
// CALLBACKS
// =============================================================================

void BearingOnlyVision::parameters_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 9) {
        RCLCPP_WARN(this->get_logger(),
            "parameters_callback: expected >= 9 values, got %zu. Ignoring.", msg->data.size());
        return;
    }

    k0_x_ = msg->data[0]; k1_x_ = msg->data[1]; k2_x_ = msg->data[2];
    k0_y_ = msg->data[3]; k1_y_ = msg->data[4]; k2_y_ = msg->data[5];
    k0_z_ = msg->data[6]; k1_z_ = msg->data[7]; k2_z_ = msg->data[8];

    reinitialize_pid_bearing_controllers();

    const bool shared_xy = (k0_x_ == k0_y_) && (k1_x_ == k1_y_) && (k2_x_ == k2_y_);
    RCLCPP_INFO(this->get_logger(),
        "Bearing PID gains updated%s: "
        "X[k0=%.3f, k1=%.3f, k2=%.5f]  "
        "Y[k0=%.3f, k1=%.3f, k2=%.5f]  "
        "Z[k0=%.3f, k1=%.3f, k2=%.5f]",
        shared_xy ? " (XY shared)" : "",
        k0_x_, k1_x_, k2_x_,
        k0_y_, k1_y_, k2_y_,
        k0_z_, k1_z_, k2_z_);
}

void BearingOnlyVision::state_changed_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    const State new_state = static_cast<State>(msg->data);

    if (new_state == current_state_) {
        RCLCPP_INFO(this->get_logger(), "Same state (%d) received. No action.", msg->data);
        return;
    }

    RCLCPP_INFO(this->get_logger(), "State transition: %d -> %d",
                static_cast<int>(current_state_), static_cast<int>(new_state));

    enable_motors_ = false;
    automatic_start_time_initialized_ = false;
    automatic_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    for (const auto& name : robot_names_) {
        takeoff_complete_[name] = false;
    }

    for (const auto& [robot_name, axes] : pids_map_) {
        for (const auto& [axis, pid] : axes) {
            pid->reset_integrator();
        }
    }
    for (const auto& [robot_name, axes] : pid_bearing_controllers_map_) {
        for (const auto& [axis, pid] : axes) {
            pid->reset_integrator();
        }
    }

    const rclcpp::Time zero_time(0, 0, get_clock()->get_clock_type());
    for (const auto& name : robot_names_) {
        last_valid_linear_commands_[name] = Eigen::Vector3d::Zero();
        last_valid_bearing_time_[name]    = zero_time;
        follower_velocity_commands_[name] = Eigen::Vector3d::Zero();
        robot_velocities_[name]           = Eigen::Vector3d::Zero();
        last_velocity_time_[name]         = this->now();
    }

    geometry_msgs::msg::Twist zero_twist;
    for (const auto& [name, pub] : cmd_vel_pubs_) {
        pub->publish(zero_twist);
    }

    switch (new_state)
    {
        case State::IDLE:
            RCLCPP_INFO(this->get_logger(), "State: IDLE");
            break;

        case State::AUTOMATIC:
            RCLCPP_INFO(this->get_logger(), "State: AUTOMATIC");
            enable_motors_ = true;
            for (const auto& leader_name : leader_names_) {
                if (robot_poses_.count(leader_name)) {
                    leader_automatic_start_pos_[leader_name] = robot_poses_[leader_name];
                }
            }
            break;

        case State::TAKING_OFF:
            RCLCPP_INFO(this->get_logger(), "State: TAKING_OFF");
            enable_motors_ = true;
            break;

        case State::LANDING:
            RCLCPP_INFO(this->get_logger(), "State: LANDING");
            enable_motors_ = true;
            break;

        case State::EMERGENCY_STOP:
            RCLCPP_INFO(this->get_logger(), "State: EMERGENCY_STOP — motors disabled.");
            enable_motors_ = false;
            break;

        default:
            RCLCPP_WARN(this->get_logger(), "Unknown state received: %d", msg->data);
            break;
    }

    current_state_ = new_state;

    std_msgs::msg::Bool enable_msg;
    enable_msg.data = enable_motors_;
    for (const auto& [name, pub] : enable_pubs_) {
        pub->publish(enable_msg);
    }
    RCLCPP_INFO(this->get_logger(), "Motor enable broadcast: %s",
                enable_motors_ ? "true" : "false");
}

void BearingOnlyVision::pose_callback(
    const geometry_msgs::msg::Pose::SharedPtr msg, const std::string& robot_name)
{
    if (!robot_poses_.count(robot_name)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Pose received for unknown robot '%s'. Ignoring.", robot_name.c_str());
        return;
    }

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(
        msg->orientation.x, msg->orientation.y,
        msg->orientation.z, msg->orientation.w)).getRPY(roll, pitch, yaw);

    robot_orientations_[robot_name] = Eigen::Vector3d(roll, pitch, yaw);
    robot_poses_[robot_name]        = Eigen::Vector3d(
        msg->position.x, msg->position.y, msg->position.z);
    last_pose_time_[robot_name]     = this->now();
}

void BearingOnlyVision::velocity_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg, const std::string& robot_name)
{
    if (!robot_velocities_.count(robot_name)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Velocity received for unknown robot '%s'. Ignoring.", robot_name.c_str());
        return;
    }

    robot_velocities_[robot_name]   = Eigen::Vector3d(
        msg->linear.x, msg->linear.y, msg->linear.z);
    last_velocity_time_[robot_name] = this->now();

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Velocity %s: [%.3f, %.3f, %.3f]",
        robot_name.c_str(),
        robot_velocities_[robot_name].x(),
        robot_velocities_[robot_name].y(),
        robot_velocities_[robot_name].z());
}

void BearingOnlyVision::vision_bearings_callback(
    const uav_msgs::msg::BearingMeasurementsArray::SharedPtr msg)
{
    for (const auto& measurement : msg->measurements)
    {
        const int obs_idx = measurement.observer_id;
        const int tgt_idx = measurement.target_id;

        if (!robot_idx_to_name_.count(obs_idx)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Observer ID %d not mapped to a robot name. Ignoring.", obs_idx);
            continue;
        }
        if (!robot_idx_to_name_.count(tgt_idx)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Target ID %d not mapped to a robot name. Ignoring.", tgt_idx);
            continue;
        }

        const std::string& obs_name = robot_idx_to_name_.at(obs_idx);
        const std::string& tgt_name = robot_idx_to_name_.at(tgt_idx);

        const Eigen::Vector3f g_raw(
            measurement.bearing_vector.x,
            measurement.bearing_vector.y,
            measurement.bearing_vector.z);

        const float norm = g_raw.norm();
        camera_bearings_[obs_name][tgt_name] =
            (norm > 1e-6f) ? Eigen::Vector3f(g_raw / norm) : Eigen::Vector3f::Zero();

        // Guardamos bearing_vector_dot como fallback para camera_bearing_dot_
        // (útil si /vision_bearings_dot no ha llegado todavía para este par)
        camera_bearing_derivatives_[obs_name][tgt_name] = Eigen::Vector3f(
            measurement.bearing_vector_dot.x,
            measurement.bearing_vector_dot.y,
            measurement.bearing_vector_dot.z);

        last_valid_bearing_time_[obs_name] = this->now();
        bearing_stamps_[obs_name][tgt_name] = this->now();
    }
}

// ── NUEVO callback ────────────────────────────────────────────────────────────
/**
 * @brief Recibe /vision_bearings_dot y almacena ḃ en camera_bearing_dot_.
 *
 * En este topic bearing_vector = ḃ (derivada estimada por el KF).
 * Se guarda directamente sin normalizar porque ḃ ⊥ b y su módulo
 * tiene unidades de [rad/s], no es unitario.
 */
void BearingOnlyVision::vision_bearings_dot_callback(
    const uav_msgs::msg::BearingMeasurementsArray::SharedPtr msg)
{
    for (const auto& measurement : msg->measurements)
    {
        const int obs_idx = measurement.observer_id;
        const int tgt_idx = measurement.target_id;

        if (!robot_idx_to_name_.count(obs_idx) || !robot_idx_to_name_.count(tgt_idx)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "vision_bearings_dot: unknown obs_id=%d or tgt_id=%d. Ignoring.",
                obs_idx, tgt_idx);
            continue;
        }

        const std::string& obs_name = robot_idx_to_name_.at(obs_idx);
        const std::string& tgt_name = robot_idx_to_name_.at(tgt_idx);

        camera_bearing_dot_[obs_name][tgt_name] = Eigen::Vector3f(
            measurement.bearing_vector.x,
            measurement.bearing_vector.y,
            measurement.bearing_vector.z);
    }
}
// ─────────────────────────────────────────────────────────────────────────────

// =============================================================================
// BEARING ERROR COMPUTATION & PUBLISHING
// =============================================================================

Eigen::MatrixXd BearingOnlyVision::calculate_bearing_errors()
{
    const size_t n = robot_names_.size();
    Eigen::MatrixXd errors(n, n * 3);
    errors.setZero();

    for (size_t i = 0; i < n; ++i)
    {
        const std::string& obs_name = robot_names_[i];

        if (!robot_orientations_.count(obs_name)) continue;

        const double yaw = robot_orientations_.at(obs_name).z();
        Eigen::Matrix3d R_global_to_robot;
        R_global_to_robot << std::cos(yaw),  std::sin(yaw), 0.0,
                            -std::sin(yaw),  std::cos(yaw), 0.0,
                             0.0,            0.0,           1.0;

        for (size_t j = 0; j < n; ++j)
        {
            if (i == j) continue;
            if (adjacency_matrix_(i, j) != 1) continue;

            const std::string& tgt_name = robot_names_[j];

            if (!camera_bearings_.count(obs_name) ||
                !camera_bearings_.at(obs_name).count(tgt_name)) continue;

            const Eigen::Vector3f& g_cam_f = camera_bearings_.at(obs_name).at(tgt_name);
            if (!g_cam_f.allFinite() || g_cam_f.norm() < 1e-6f) continue;

            const Eigen::Vector3d gij_c =
                R_camera_to_robot_ * g_cam_f.cast<double>().normalized();

            if (!gijA_d_.count(obs_name) || !gijA_d_.at(obs_name).count(tgt_name)) continue;
            const Eigen::Vector3d gij_d =
                (R_global_to_robot * gijA_d_.at(obs_name).at(tgt_name)).normalized();

            const Eigen::Vector3d error = gij_c - gij_d;

            const size_t col = j * 3;
            errors(i, col + 0) = error.x();
            errors(i, col + 1) = error.y();
            errors(i, col + 2) = error.z();
        }
    }
    return errors;
}

void BearingOnlyVision::calculate_and_publish_bearing_errors()
{
    const Eigen::MatrixXd errors = calculate_bearing_errors();

    std_msgs::msg::Float32MultiArray msg;
    msg.data.reserve(errors.size());
    for (int i = 0; i < errors.rows(); ++i)
        for (int j = 0; j < errors.cols(); ++j)
            msg.data.push_back(static_cast<float>(errors(i, j)));

    auto make_dim = [](const std::string& label, uint32_t size, uint32_t stride) {
        std_msgs::msg::MultiArrayDimension d;
        d.label = label; d.size = size; d.stride = stride;
        return d;
    };
    msg.layout.dim.push_back(make_dim("observer_idx", errors.rows(),
                                       errors.rows() * errors.cols()));
    msg.layout.dim.push_back(make_dim("target_idx",   robot_names_.size(), 3));
    msg.layout.dim.push_back(make_dim("xyz_error",    3, 1));
    msg.layout.data_offset = 0;

    bearing_error_pub_->publish(msg);
}

void BearingOnlyVision::calculate_and_publish_rms_bearing_error(
    const Eigen::MatrixXd& errors)
{
    double sum_sq = 0.0;
    int    count  = 0;

    for (int i = 0; i < errors.rows(); ++i) {
        for (int j = 0; j < errors.cols() / 3; ++j) {
            if (i == j) continue;
            if (adjacency_matrix_(i, j) != 1) continue;

            const int col = j * 3;
            if (col + 2 < errors.cols()) {
                sum_sq += errors(i, col)     * errors(i, col);
                sum_sq += errors(i, col + 1) * errors(i, col + 1);
                sum_sq += errors(i, col + 2) * errors(i, col + 2);
                count  += 3;
            }
        }
    }

    std_msgs::msg::Float32 rms_msg;
    rms_msg.data = (count > 0) ? static_cast<float>(std::sqrt(sum_sq / count)) : 0.0f;
    rms_bearing_error_pub_->publish(rms_msg);
}

// =============================================================================
// VISUALIZATION
// =============================================================================

void BearingOnlyVision::publish_visualization_markers()
{
    auto marker_array = std::make_shared<visualization_msgs::msg::MarkerArray>();
    int  id = 0;
    const rclcpp::Time now = this->now();

    for (const auto& [name, pose] : robot_poses_)
    {
        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id = "world";
        sphere.header.stamp    = now;
        sphere.ns              = "robot_spheres";
        sphere.id              = id++;
        sphere.type            = visualization_msgs::msg::Marker::SPHERE;
        sphere.action          = visualization_msgs::msg::Marker::ADD;
        sphere.pose.position.x = pose.x();
        sphere.pose.position.y = pose.y();
        sphere.pose.position.z = pose.z();
        sphere.pose.orientation.w = 1.0;
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.15;
        sphere.color   = robot_colors_.at(name);
        marker_array->markers.push_back(sphere);

        visualization_msgs::msg::Marker label;
        label.header        = sphere.header;
        label.ns            = "robot_names";
        label.id            = id++;
        label.type          = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.action        = visualization_msgs::msg::Marker::ADD;
        label.pose.position.x = pose.x();
        label.pose.position.y = pose.y();
        label.pose.position.z = pose.z() + 0.2;
        label.pose.orientation.w = 1.0;
        label.scale.z       = 0.1;
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0f;
        label.text          = name;
        marker_array->markers.push_back(label);
    }

    for (size_t i = 0; i < robot_names_.size(); ++i) {
        const std::string& obs = robot_names_[i];
        if (!robot_poses_.count(obs)) continue;

        for (size_t j = 0; j < robot_names_.size(); ++j) {
            if (i == j) continue;
            if (adjacency_matrix_(i, j) != 1) continue;

            const std::string& tgt = robot_names_[j];
            if (!robot_poses_.count(tgt)) continue;

            const Eigen::Vector3d& p0 = robot_poses_.at(obs);
            const Eigen::Vector3d& p1 = robot_poses_.at(tgt);

            const double time_since_bearing =
                (now - last_valid_bearing_time_[obs]).seconds();
            const float alpha =
                (time_since_bearing > bearing_loss_timeout_sec_) ? 0.2f :
                (time_since_bearing > bearing_loss_timeout_sec_ / 2.0) ? 0.5f : 0.7f;

            visualization_msgs::msg::Marker line;
            line.header.frame_id = "world";
            line.header.stamp    = now;
            line.ns              = "communication_links";
            line.id              = id++;
            line.type            = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action          = visualization_msgs::msg::Marker::ADD;
            line.scale.x         = 0.02;
            line.color           = robot_colors_.at(obs);
            line.color.a         = alpha;

            geometry_msgs::msg::Point pt0, pt1;
            pt0.x = p0.x(); pt0.y = p0.y(); pt0.z = p0.z();
            pt1.x = p1.x(); pt1.y = p1.y(); pt1.z = p1.z();
            line.points.push_back(pt0);
            line.points.push_back(pt1);
            marker_array->markers.push_back(line);
        }
    }

    visualization_pub_->publish(*marker_array);
}

// =============================================================================
// CONTROL LOOP
// =============================================================================

void BearingOnlyVision::control_loop()
{
    const rclcpp::Time current_time = this->now();

    if (last_control_loop_time_.nanoseconds() == 0) {
        real_dt_ = 1.0 / control_loop_rate_;
    } else {
        real_dt_ = (current_time - last_control_loop_time_).seconds();
        const double nominal_dt = 1.0 / control_loop_rate_;
        if (real_dt_ <= 0.0 || real_dt_ > 1.0) {
            RCLCPP_WARN(this->get_logger(),
                "Abnormal real_dt=%.6f s detected. Falling back to nominal dt.", real_dt_);
            real_dt_ = nominal_dt;
        } else if (real_dt_ > 2.5 * nominal_dt) {
            // Gazebo catch-up: physics missed ≥2 steps. Clamping dt prevents
            // velocity command spikes (follower_velocity += accel * real_dt_).
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "dt spike %.4f s (>2.5x nominal %.4f s). Clamping to nominal.",
                real_dt_, nominal_dt);
            real_dt_ = nominal_dt;
        }
    }
    last_control_loop_time_ = current_time;

    const Eigen::MatrixXd current_bearing_errors = calculate_bearing_errors();
    calculate_and_publish_bearing_errors();
    calculate_and_publish_rms_bearing_error(current_bearing_errors);
    publish_visualization_markers();

    if (robot_names_.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "No robots configured.");
        return;
    }

    if (!enable_motors_) {
        geometry_msgs::msg::Twist zero;
        std_msgs::msg::Bool disable_msg;
        disable_msg.data = false;
        for (const auto& [name, pub] : cmd_vel_pubs_) pub->publish(zero);
        for (const auto& [name, pub] : enable_pubs_)  pub->publish(disable_msg);
        return;
    }

    // The enable message of a state change can be lost in the bridge, and then
    // the drones ignore the commands. Repeat it while the motors should run.
    {
        std_msgs::msg::Bool enable_msg;
        enable_msg.data = true;
        for (const auto& [name, pub] : enable_pubs_) pub->publish(enable_msg);
    }

    for (const auto& name : robot_names_) {
        if (!robot_poses_.count(name) ||
            !robot_orientations_.count(name) ||
            !robot_velocities_.count(name))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Waiting for data from all robots...");
            return;
        }
        const double pose_age = (current_time - last_pose_time_[name]).seconds();
        if (pose_age > POSE_TIMEOUT_SEC) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Stale pose for '%s': %.3f s old.", name.c_str(), pose_age);
            geometry_msgs::msg::Twist zero;
            for (const auto& [n, pub] : cmd_vel_pubs_) pub->publish(zero);
            return;
        }
    }

    try {
        if (current_state_ == State::AUTOMATIC && !automatic_start_time_initialized_) {
            automatic_start_time_             = current_time;
            automatic_start_time_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "AUTOMATIC timer started at %.2f s.",
                        automatic_start_time_.seconds());
        }

        const double elapsed = (automatic_start_time_.nanoseconds() > 0)
                                ? (current_time - automatic_start_time_).seconds()
                                : 0.0;

        if (current_state_ == State::AUTOMATIC && elapsed >= automatic_duration_sec_) {
            RCLCPP_INFO(this->get_logger(),
                "AUTOMATIC duration reached (%.2f s). Transitioning to LANDING.", elapsed);
            auto land_msg = std::make_shared<std_msgs::msg::Int32>();
            land_msg->data = static_cast<int>(State::LANDING);
            state_changed_callback(land_msg);
            return;
        }

        std::map<std::string, geometry_msgs::msg::Twist> cmd_vels;
        bool all_landed = true;

        for (const auto& robot_name : robot_names_)
        {
            const Eigen::Vector3d& current_pos = robot_poses_.at(robot_name);
            const Eigen::Vector3d& current_ori = robot_orientations_.at(robot_name);
            geometry_msgs::msg::Twist cmd;

            const double yaw_target = get_yaw_target(robot_name);

            switch (current_state_)
            {
                // -----------------------------------------------------------------
                case State::TAKING_OFF:
                {
                    const double cz = current_pos.z();
                    const double tx = initial_xy_takeoff_positions_[robot_name].x();
                    const double ty = initial_xy_takeoff_positions_[robot_name].y();

                    const double ex_global = tx - current_pos.x();
                    const double ey_global = ty - current_pos.y();

                    const double yaw = current_ori.z();
                    const double ex_body =  ex_global * std::cos(yaw) + ey_global * std::sin(yaw);
                    const double ey_body = -ex_global * std::sin(yaw) + ey_global * std::cos(yaw);

                    cmd.linear.x = pids_map_[robot_name]["x"]->update(0.0, ex_body, real_dt_);
                    cmd.linear.y = pids_map_[robot_name]["y"]->update(0.0, ey_body, real_dt_);
                    // Rise first: a sideways command while on the ground keeps the
                    // drone from lifting off after it landed away from its start.
                    if (cz < 0.2) {
                        cmd.linear.x = 0.0;
                        cmd.linear.y = 0.0;
                    }

                    if (std::abs(cz - takeoff_height_) <= takeoff_height_tolerance_) {
                        cmd.linear.z = 0.0;
                        if (!takeoff_complete_[robot_name]) {
                            RCLCPP_INFO(this->get_logger(),
                                "Takeoff complete for %s at z=%.2f m.", robot_name.c_str(), cz);
                            takeoff_complete_[robot_name] = true;
                        }
                    } else {
                        cmd.linear.z = pids_map_[robot_name]["z"]->update(cz, takeoff_height_, real_dt_);
                        takeoff_complete_[robot_name] = false;
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                            "TAKING_OFF %s | pos(%.2f,%.2f,%.2f) tgt(%.2f,%.2f,%.2f)",
                            robot_name.c_str(),
                            current_pos.x(), current_pos.y(), cz,
                            tx, ty, takeoff_height_);
                    }

                    if (takeoff_complete_[robot_name]) {
                        const double yaw_error = wrap_angle(current_ori.z() - yaw_target);
                        cmd.angular.z = pids_map_[robot_name]["yaw"]->update(yaw_error, 0.0, real_dt_);
                    } else {
                        cmd.angular.z = 0.0;
                    }
                    break;
                }

                // -----------------------------------------------------------------
                case State::AUTOMATIC:
                {
                    if (is_leader(robot_name))
                    {
                        constexpr double a           = 2.0;
                        constexpr double base_omega  = 0.1;
                        constexpr double speed_factor = 1.5;
                        const double omega = base_omega * speed_factor;
                        const double t_traj = elapsed * trajectory_time_scale_;
                        const double theta = omega * t_traj + M_PI / 2.0;
                        const double denom = 1.0 + std::pow(std::sin(theta), 2);

                        const double x_desired = (a * std::cos(theta)) / denom
                                                 + initial_xy_takeoff_positions_[robot_name].x();
                        const double y_desired = (a * std::sin(theta) * std::cos(theta)) / denom
                                                 + initial_xy_takeoff_positions_[robot_name].y();
                        const double z_desired = takeoff_height_ + 0.4 * t_traj;

                        cmd.linear.x = pids_map_[robot_name]["x"]->update(current_pos.x(), x_desired, real_dt_) * 0.2;
                        cmd.linear.y = pids_map_[robot_name]["y"]->update(current_pos.y(), y_desired, real_dt_) * 0.2;
                        cmd.linear.z = pids_map_[robot_name]["z"]->update(current_pos.z(), z_desired, real_dt_) * 0.2;

                        {
                            const double yaw_error = wrap_angle(current_ori.z() - yaw_target);
                            cmd.angular.z = std::clamp(
                                pids_map_[robot_name]["yaw"]->update(yaw_error, 0.0, real_dt_),
                                -max_angular_velocity_, max_angular_velocity_);
                        }
                    }
                    else  // Follower: vision bearing-based control
                    {
                        Eigen::Vector3d desired_accel = Eigen::Vector3d::Zero();
                        int valid_bearings = 0;

                        const double yaw = current_ori.z();
                        Eigen::Matrix3d R_global_to_robot;
                        R_global_to_robot << std::cos(yaw),  std::sin(yaw), 0.0,
                                            -std::sin(yaw),  std::cos(yaw), 0.0,
                                             0.0,            0.0,           1.0;

                        for (const auto& tgt_name : robot_names_)
                        {
                            if (robot_name == tgt_name) continue;

                            const int obs_idx = robot_name_to_idx_.at(robot_name);
                            const int tgt_idx = robot_name_to_idx_.at(tgt_name);
                            if (adjacency_matrix_(obs_idx, tgt_idx) != 1) continue;

                            if (!camera_bearings_.count(robot_name) ||
                                !camera_bearings_.at(robot_name).count(tgt_name)) continue;

                            const Eigen::Vector3f& g_cam_f =
                                camera_bearings_.at(robot_name).at(tgt_name);
                            if (!g_cam_f.allFinite() || g_cam_f.norm() < 1e-6f) continue;

                            // A pair that stopped being seen must not keep driving the
                            // integrated velocity command with its last measurement.
                            const auto obs_stamps = bearing_stamps_.find(robot_name);
                            if (obs_stamps == bearing_stamps_.end() ||
                                !obs_stamps->second.count(tgt_name) ||
                                (current_time - obs_stamps->second.at(tgt_name)).seconds() >=
                                    bearing_loss_timeout_sec_) continue;

                            // Bearing medido en frame robot
                            const Eigen::Vector3d gij_c =
                                (R_camera_to_robot_ * g_cam_f.cast<double>()).normalized();

                            // ── NUEVO: obtener ḃ desde camera_bearing_dot_ ────────────
                            // Prioridad: /vision_bearings_dot  →  fallback: bearing_vector_dot
                            // de /vision_bearings.
                            Eigen::Vector3d dot_gij = Eigen::Vector3d::Zero();

                            if (camera_bearing_dot_.count(robot_name) &&
                                camera_bearing_dot_.at(robot_name).count(tgt_name))
                            {
                                // Fuente primaria: topic dedicado bearing_dot
                                dot_gij = R_camera_to_robot_ *
                                    camera_bearing_dot_.at(robot_name)
                                                       .at(tgt_name).cast<double>();
                            }
                            else if (camera_bearing_derivatives_.count(robot_name) &&
                                     camera_bearing_derivatives_.at(robot_name).count(tgt_name))
                            {
                                // Fallback: campo bearing_vector_dot de /vision_bearings
                                dot_gij = R_camera_to_robot_ *
                                    camera_bearing_derivatives_.at(robot_name)
                                                               .at(tgt_name).cast<double>();
                            }
                            // ─────────────────────────────────────────────────────────

                            // Desired bearing en frame robot
                            if (!gijA_d_.count(robot_name) ||
                                !gijA_d_.at(robot_name).count(tgt_name)) continue;
                            const Eigen::Vector3d gij_d =
                                (R_global_to_robot * gijA_d_.at(robot_name).at(tgt_name))
                                    .normalized();

                            if (!pid_bearing_controllers_map_.count(robot_name)) {
                                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "No bearing PID found for '%s'.", robot_name.c_str());
                                break;
                            }

                            desired_accel.x() += pid_bearing_controllers_map_[robot_name]["x"]
                                ->update(gij_c.x(), gij_d.x(), dot_gij.x(), 0.0, real_dt_);
                            desired_accel.y() += pid_bearing_controllers_map_[robot_name]["y"]
                                ->update(gij_c.y(), gij_d.y(), dot_gij.y(), 0.0, real_dt_);
                            desired_accel.z() += pid_bearing_controllers_map_[robot_name]["z"]
                                ->update(gij_c.z(), gij_d.z(), dot_gij.z(), 0.0, real_dt_);
                            ++valid_bearings;
                        }

                        if (valid_bearings > 0) {
                            follower_velocity_commands_[robot_name] += desired_accel * real_dt_;
                        } else {
                            const double bearing_age =
                                (current_time - last_valid_bearing_time_[robot_name]).seconds();
                            if (bearing_age >= bearing_loss_timeout_sec_) {
                                follower_velocity_commands_[robot_name].setZero();
                            }
                        }

                        constexpr double FOLLOWER_GAIN = 0.3;
                        auto & v_cmd = follower_velocity_commands_[robot_name];
                        const double v_max = max_linear_velocity_ / FOLLOWER_GAIN;
                        if (v_cmd.norm() > v_max) {
                            v_cmd *= v_max / v_cmd.norm();
                        }
                        cmd.linear.x = follower_velocity_commands_[robot_name].x() * FOLLOWER_GAIN;
                        cmd.linear.y = follower_velocity_commands_[robot_name].y() * FOLLOWER_GAIN;
                        cmd.linear.z = follower_velocity_commands_[robot_name].z() * FOLLOWER_GAIN;

                        if (elapsed > 0.1) {
                            const double yaw_error = wrap_angle(current_ori.z() - yaw_target);
                            cmd.angular.z = std::clamp(
                                pids_map_[robot_name]["yaw"]->update(yaw_error, 0.0, real_dt_),
                                -max_angular_velocity_ / 2.0, max_angular_velocity_ / 2.0);
                        } else {
                            cmd.angular.z = 0.0;
                        }
                    }
                    break;
                }

                // -----------------------------------------------------------------
                case State::LANDING:
                {
                    const double cz = current_pos.z();
                    if (cz > landing_threshold_) {
                        all_landed = false;
                        cmd.linear.z = pids_map_[robot_name]["z"]->update(cz, 0.0, real_dt_) * landing_speed_factor_;
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                            "LANDING %s | z=%.2f m, cmd_z=%.2f", robot_name.c_str(), cz, cmd.linear.z);
                    }
                    break;
                }

                case State::IDLE:
                case State::EMERGENCY_STOP:
                default:
                    break;
            }

            cmd_vels[robot_name] = cmd;
        }

        // Motors stop together, once the whole formation is on the ground.
        if (current_state_ == State::LANDING && all_landed) {
            RCLCPP_INFO(this->get_logger(), "Landing complete for all robots.");
            auto idle_msg = std::make_shared<std_msgs::msg::Int32>();
            idle_msg->data = static_cast<int>(State::IDLE);
            state_changed_callback(idle_msg);
            return;
        }

        for (const auto& [name, pub] : cmd_vel_pubs_) {
            if (cmd_vels.count(name)) {
                pub->publish(cmd_vels.at(name));
            }
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in control_loop: %s", e.what());
    }
}

// =============================================================================
// MAIN
// =============================================================================

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BearingOnlyVision>());
    rclcpp::shutdown();
    return 0;
}