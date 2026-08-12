/**
 * PPO Deploy Node for F1Tenth
 *
 * Runs the trained PPO policy on the physical F1TENTH car.
 * C++ scaffold that mirrors scripts/ppo_deploy_node.py — pick whichever
 * language you'd rather extend, both compute the same thing.
 * This is just a template, you are free to implement your own node!
 *
 * Requires:
 *   - ONNX Runtime C++ (https://onnxruntime.ai — install a prebuilt release,
 *     or `libonnxruntime-dev` if your distro packages it)
 *   - nlohmann/json (`sudo apt install nlohmann-json3-dev`)
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <vector>

#include <nlohmann/json.hpp>
#include <onnxruntime_cxx_api.h>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rl_ppo/rl_ppo_utils.hpp"

using namespace rl_ppo;
using json = nlohmann::json;

class PPODeployNode : public rclcpp::Node {
public:
    PPODeployNode()
        : Node("ppo_deploy_node"),
          env_(ORT_LOGGING_LEVEL_WARNING, "ppo_deploy_node") {
        // ── Parameters ────────────────────────────────────────────────────
        declare_parameter("onnx_path", "");
        declare_parameter("config_path", "");
        declare_parameter("scan_topic", "/scan");
        declare_parameter("odom_topic", "/odom");
        declare_parameter("drive_topic", "/drive");
        declare_parameter("max_speed_cap", 2.0);
        declare_parameter("joy_arm_button", 7);  // RB on 8BitDo

        std::string onnx_path = get_parameter("onnx_path").as_string();
        std::string config_path = get_parameter("config_path").as_string();
        config_.max_speed_cap = get_parameter("max_speed_cap").as_double();
        config_.joy_arm_button = static_cast<int>(get_parameter("joy_arm_button").as_int());

        // ── Load config written by export_ppo_onnx.py ────────────────────
        // Contains all scaling constants from the training wrapper —
        // max_speed, min_speed, max_steer, lidar_clip, num_beams, fov
        std::ifstream f(config_path);
        json cfg;
        f >> cfg;

        config_.num_beams = cfg.at("num_beams").get<int>();
        config_.lidar_clip = cfg.at("lidar_clip").get<double>();
        config_.max_speed = cfg.at("max_speed").get<double>();
        config_.min_speed = cfg.at("min_speed").get<double>();
        config_.max_steer = cfg.at("max_steer").get<double>();
        config_.lidar_fov_rad = cfg.value("lidar_fov_rad", 4.7);
        config_.control_rate_hz = cfg.value("control_rate_hz", 100.0);

        // ── Load ONNX model ───────────────────────────────────────────────
        Ort::SessionOptions session_options;
        session_ = std::make_unique<Ort::Session>(env_, onnx_path.c_str(), session_options);

        RCLCPP_INFO(get_logger(),
            "PPO policy loaded | beams=%d | speed=[%.2f,%.2f] | steer=+-%.4f | cap=%.2f m/s",
            config_.num_beams, config_.min_speed, config_.max_speed,
            config_.max_steer, config_.max_speed_cap);

        // Sensor topics use BEST_EFFORT — they drop packets rather than queue.
        // This keeps latency low (we want the freshest scan, not an old one)
        auto sensor_qos = rclcpp::SensorDataQoS();

        // ═════════════════════════════════════════════════════════════════
        //  TODO 1 — Create Subscribers
        // ═════════════════════════════════════════════════════════════════
        // Wire the node up to the car's sensor topics. Same 4 topics as the
        // Python node:
        //
        // (a) Lidar scan — the car's main sensor
        //     Topic:    get_parameter("scan_topic").as_string()  -> "/scan"
        //     Msg type: sensor_msgs::msg::LaserScan
        //     Callback: &PPODeployNode::scanCallback
        //     QoS:      sensor_qos
        //
        // (b) Odometry — gives us the car's actual speed
        //     Topic:    get_parameter("odom_topic").as_string()  -> "/odom"
        //     Msg type: nav_msgs::msg::Odometry
        //     Callback: &PPODeployNode::odomCallback
        //     QoS:      sensor_qos
        //
        // (c) Servo position — gives us the car's actual steering angle
        //     Topic:    "/sensors/servo_position_command"
        //     Msg type: sensor_msgs::msg::LaserScan (VESC node reuses this msg type)
        //     Callback: &PPODeployNode::servoCallback
        //     QoS:      10 (reliable, small queue)
        //
        // (d) Joystick — RB button arms/disarms autonomous driving
        //     Topic:    "/joy"
        //     Msg type: sensor_msgs::msg::Joy
        //     Callback: &PPODeployNode::joyCallback
        //     QoS:      10
        //
        // Reference:
        //   scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        //       topic, qos,
        //       std::bind(&PPODeployNode::scanCallback, this, std::placeholders::_1));
        // ─────────────────────────────────────────────────────────────────
        // TODO 1a: Subscribe to /scan  -> scan_sub_
        // TODO 1b: Subscribe to /odom  -> odom_sub_
        // TODO 1c: Subscribe to /sensors/servo_position_command -> servo_sub_
        // TODO 1d: Subscribe to /joy   -> joy_sub_
        
        
        
        // ═════════════════════════════════════════════════════════════════

        // ═════════════════════════════════════════════════════════════════
        //   TODO 2 — Create Drive Publisher
        // ═════════════════════════════════════════════════════════════════
        // Create a publisher that sends drive commands to the car.
        // The ackermann_mux will relay this to the VESC.
        //
        //     Msg type:  ackermann_msgs::msg::AckermannDriveStamped
        //     Topic:     get_parameter("drive_topic").as_string()  -> "/drive"
        //     QoS depth: 10
        //
        // Store it as drive_pub_ so publishDrive() can use it.
        //
        // Reference:
        //   drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        //       topic, depth);
        // ─────────────────────────────────────────────────────────────────
        // TODO 2: create drive_pub_
        
        
        
        
        // ═════════════════════════════════════════════════════════════════

        control_timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / config_.control_rate_hz),
            std::bind(&PPODeployNode::controlCallback, this));

        RCLCPP_INFO(get_logger(), "Running at %.0f Hz", config_.control_rate_hz);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;
        if (beam_idx_.empty()) {
            beam_idx_ = computeBeamIndices(*msg, config_);
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        speed_meas_ = std::sqrt(vx * vx + vy * vy);
    }

    void servoCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // /sensors/servo_position_command carries servo position [0,1].
        // We invert it back to steering radians using ackermann_to_vesc params.
        // config_.servo_gain / servo_offset must match your ackermann_to_vesc config.
        if (!msg->ranges.empty()) {
            double servo_pos = msg->ranges[0];
            steer_echo_ = (servo_pos - config_.servo_offset) / config_.servo_gain;
        }
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // RB button — same as every other autonomous mode
        if (config_.joy_arm_button < static_cast<int>(msg->buttons.size())) {
            joy_armed_ = msg->buttons[config_.joy_arm_button] != 0;
        }
    }

    /**
     * Build the (num_beams + 2)-dim observation exactly as in training.
     * This must match F1TenthRLWrapper._process_obs() identically —
     * if it differs, the policy will see different inputs than it trained on.
     */
    std::vector<float> buildObs(const sensor_msgs::msg::LaserScan& scan) {
        // Sanitize — real lidar returns inf/nan for out-of-range readings.
        // The sim never does this, so we must handle it here.
        std::vector<float> raw_beams(config_.num_beams);
        for (int i = 0; i < config_.num_beams; ++i) {
            float r = scan.ranges[beam_idx_[i]];
            raw_beams[i] = (std::isfinite(r) && r > scan.range_min)
                ? r
                : static_cast<float>(config_.lidar_clip);
        }

        // ═════════════════════════════════════════════════════════════════
        //  TODO 3 — Normalize the Observation
        // ═════════════════════════════════════════════════════════════════
        // The policy was trained on observations normalized to [0, 1].
        // If you pass raw values here, the policy sees completely different
        // numbers than it trained on and will behave randomly.
        //
        // You need to normalize 3 things — same formulas as training:
        //
        // (a) Lidar beams — clip to max range then divide
        //     Hint: clamp(raw_beams[i], 0.0, config_.lidar_clip) / config_.lidar_clip
        //
        // (b) Speed — divide by max speed, clip to [0, 1]
        //     speed_meas_ is in m/s, config_.max_speed is the training limit
        //     Hint: clamp(speed_meas_ / config_.max_speed, 0.0, 1.0)
        //
        // (c) Steering angle — map [-max_steer, +max_steer] -> [0, 1]
        //     steer_echo_ is in radians, config_.max_steer is the limit
        //     Hint: clamp((steer_echo_ + config_.max_steer) /
        //                 (2 * config_.max_steer), 0.0, 1.0)
        //
        // Fill obs as [beams_normalized..., speed_n, steer_n].
        // Length must be num_beams + 2.
        // ─────────────────────────────────────────────────────────────────
        std::vector<float> obs(config_.num_beams + 2, 0.0f);
        // TODO 3a: normalize raw_beams into obs[0 .. num_beams-1]
        // TODO 3b: normalize speed_meas_ into obs[num_beams]
        // TODO 3c: normalize steer_echo_ into obs[num_beams + 1]
        
        
        
        // ═════════════════════════════════════════════════════════════════

        return obs;
    }

    void publishDrive(double steer, double speed) {
        if (!drive_pub_) return;
        ackermann_msgs::msg::AckermannDriveStamped msg;
        msg.header.stamp = get_clock()->now();
        msg.drive.steering_angle = steer;
        msg.drive.speed = speed;
        drive_pub_->publish(msg);
    }

    void controlCallback() {
        // Called at control_rate_hz — this is the main inference loop.
        // Mirrors the training loop: obs -> policy -> action -> drive command.

        // No scan yet — wait for first message
        if (!latest_scan_ || beam_idx_.empty()) return;

        // RB not held — publish stop so mux sees an active command
        if (!joy_armed_) {
            publishDrive(0.0, 0.0);
            return;
        }

        std::vector<float> obs = buildObs(*latest_scan_);

        std::array<int64_t, 2> input_shape{1, static_cast<int64_t>(obs.size())};
        Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            mem_info, obs.data(), obs.size(), input_shape.data(), input_shape.size());

        Ort::AllocatorWithDefaultOptions allocator;
        auto input_name = session_->GetInputNameAllocated(0, allocator);
        auto output_name = session_->GetOutputNameAllocated(0, allocator);
        const char* input_names[] = {input_name.get()};
        const char* output_names[] = {output_name.get()};

        auto output_tensors = session_->Run(
            Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);
        const float* action = output_tensors.front().GetTensorData<float>();

        double steer, speed;
        scaleAction(action, config_, steer, speed);
        speed = std::min(speed, config_.max_speed_cap);
        last_steer_ = steer;

        publishDrive(steer, speed);
    }

    // Configuration
    Config config_;

    // ONNX Runtime
    Ort::Env env_;
    std::unique_ptr<Ort::Session> session_;

    // Beam index lookup (computed once on first scan)
    std::vector<int> beam_idx_;

    // State
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    double speed_meas_ = 0.0;
    double steer_echo_ = 0.0;   // from /sensors/servo_position_command
    double last_steer_ = 0.0;   // last commanded steer
    bool joy_armed_ = false;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr servo_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PPODeployNode>());
    rclcpp::shutdown();
    return 0;
}
