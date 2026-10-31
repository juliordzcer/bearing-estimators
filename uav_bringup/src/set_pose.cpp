// Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
// SPDX-License-Identifier: Apache-2.0
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // For tf2::toMsg and tf2::fromMsg
#include <cmath>
#include <optional>
#include <vector>
#include <string>
#include <nlohmann/json.hpp> // For JSON parsing

// Alias for convenience
using json = nlohmann::json;

class MultiRobotPosePublisher : public rclcpp::Node
{
public:
    MultiRobotPosePublisher() : Node("multi_robot_pose_publisher")
    {
        // Declare parameters with default generic values (empty JSON strings)
        this->declare_parameter<std::string>("robot_names", "[]");
        this->declare_parameter<std::string>("initial_conditions", "[]");

        // Get and parse parameters
        try {
            std::string robot_names_str = this->get_parameter("robot_names").as_string();
            std::string initial_conditions_str = this->get_parameter("initial_conditions").as_string();

            robot_names_ = json::parse(robot_names_str).get<std::vector<std::string>>();
            initial_conditions_ = json::parse(initial_conditions_str).get<std::vector<std::vector<double>>>();

        } catch (const json::parse_error& e) {
            RCLCPP_ERROR(this->get_logger(), "JSON Parse ERROR: %s", e.what());
            throw; // Re-throw to terminate node if parsing fails
        } catch (const rclcpp::ParameterTypeException& e) {
            RCLCPP_ERROR(this->get_logger(), "Parameter Type ERROR: %s", e.what());
            throw; // Re-throw if parameter type is incorrect
        }

        // Validate lengths match
        if (robot_names_.size() != initial_conditions_.size()) {
            RCLCPP_ERROR(this->get_logger(),
                         "Number of robots (%zu) does not match initial conditions (%zu)",
                         robot_names_.size(), initial_conditions_.size());
            throw std::runtime_error("Parameter lengths do not match");
        }

        // The pose message is lost if the bridge is not connected yet, and all
        // drones would stay stacked at the origin. Keep sending it until the
        // poses reported by Gazebo show that every drone is in place.
        reported_.resize(robot_names_.size());
        for (size_t i = 0; i < robot_names_.size(); ++i) {
            pose_subs_.push_back(this->create_subscription<geometry_msgs::msg::Pose>(
                "/" + robot_names_[i] + "/pose", rclcpp::SensorDataQoS(),
                [this, i](const geometry_msgs::msg::Pose::SharedPtr msg) { reported_[i] = *msg; }));
        }

        // Create publishers dynamically
        for (const auto& name : robot_names_) {
            publishers_.push_back(this->create_publisher<geometry_msgs::msg::Pose>(name + "/set_pose", 10));
        }

        // One-shot timer. create_timer (not create_wall_timer) uses the node
        // clock, so with use_sim_time this delay is measured in simulated
        // seconds instead of wall-clock seconds.
        timer_ = this->create_timer(
            std::chrono::duration<double>(2.0), // 2.0 seconds
            std::bind(&MultiRobotPosePublisher::publishInitialPoses, this));

        RCLCPP_INFO(this->get_logger(), "MultiRobotPosePublisher node initialized.");
    }

private:
    std::vector<std::string> robot_names_;
    std::vector<std::vector<double>> initial_conditions_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr> publishers_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr> pose_subs_;
    std::vector<std::optional<geometry_msgs::msg::Pose>> reported_;
    int attempts_ = 0;
    static constexpr int kMaxAttempts = 15;
    static constexpr double kPoseToleranceM = 0.05;

    bool allInPlace() const
    {
        for (size_t i = 0; i < robot_names_.size(); ++i) {
            if (!reported_[i]) {
                return false;
            }
            const double dx = reported_[i]->position.x - initial_conditions_[i][0];
            const double dy = reported_[i]->position.y - initial_conditions_[i][1];
            if (std::hypot(dx, dy) > kPoseToleranceM) {
                return false;
            }
        }
        return true;
    }

    double hd_ = 0.05075; // Takeoff height (adjustable)

    void publishInitialPoses()
    {
        /**
         * @brief Publishes initial poses until every drone reports being in place.
         */
        if (allInPlace()) {
            timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "All drones in place after %d attempts.", attempts_);
            return;
        }
        if (++attempts_ > kMaxAttempts) {
            timer_->cancel();
            RCLCPP_ERROR(this->get_logger(), "Drones did not reach their initial poses.");
            return;
        }
        
        for (size_t i = 0; i < robot_names_.size(); ++i) {
            const std::string& name = robot_names_[i];
            const std::vector<double>& conditions = initial_conditions_[i];

            if (conditions.size() < 4) {
                RCLCPP_ERROR(this->get_logger(), "Invalid initial conditions for %s: %s", name.c_str(), conditions_to_string(conditions).c_str());
                continue;
            }

            geometry_msgs::msg::Pose pose;
            pose.position.x = conditions[0];
            pose.position.y = conditions[1];
            pose.position.z = conditions[2] + hd_; // Add takeoff height

            // Convert yaw (in radians) to quaternion
            tf2::Quaternion q;
            q.setRPY(0, 0, conditions[3]); // Roll, Pitch, Yaw

            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();

            publishers_[i]->publish(pose);
            RCLCPP_INFO(this->get_logger(), "Published initial pose for %s: [%.2f, %.2f, %.2f, %.2f]",
                        name.c_str(), conditions[0], conditions[1], conditions[2], conditions[3]);
        }

    }

    // Helper to convert vector<double> to string for logging
    std::string conditions_to_string(const std::vector<double>& conditions) {
        std::string s = "[";
        for (size_t j = 0; j < conditions.size(); ++j) {
            s += std::to_string(conditions[j]);
            if (j < conditions.size() - 1) {
                s += ", ";
            }
        }
        s += "]";
        return s;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<MultiRobotPosePublisher> node = nullptr; // Initialize to nullptr
    try {
        node = std::make_shared<MultiRobotPosePublisher>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        if (node) { // Check if node was successfully created
            RCLCPP_ERROR(node->get_logger(), "ERROR: %s", e.what());
        } else {
            // If node creation failed, no logger is available. Print to stderr.
            std::cerr << "ERROR: Failed to create node: " << e.what() << std::endl;
        }
    }
    rclcpp::shutdown();
    return 0;
}