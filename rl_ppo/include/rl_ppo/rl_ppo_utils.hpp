#ifndef RL_PPO_UTILS_HPP
#define RL_PPO_UTILS_HPP

#include <algorithm>
#include <cmath>
#include <vector>

#include "sensor_msgs/msg/laser_scan.hpp"

namespace rl_ppo {

// ============================================================================
// Constants
// ============================================================================
constexpr int kActDim = 2;  // Policy output size: [steer_cmd, speed_cmd] in [-1, 1]

// ============================================================================
// Configuration - filled in from ppo_deploy_config.json (export_ppo_onnx.py)
// and ROS parameters
// ============================================================================
struct Config {
    int num_beams = 108;         // Number of lidar beams the policy expects
    double lidar_clip = 10.0;    // Max lidar range used during training [m]
    double max_speed = 6.0;      // Max speed used during training [m/s]
    double min_speed = 1.5;      // Min speed used during training [m/s]
    double max_steer = 0.4189;   // Max steering angle used during training [rad]
    double lidar_fov_rad = 4.7;  // LiDAR field of view used during training [rad]
    double control_rate_hz = 100.0;

    double max_speed_cap = 2.0;  // Hard ceiling regardless of policy output [m/s]
    int joy_arm_button = 7;      // RB on 8BitDo

    // Must match your ackermann_to_vesc config / car calibration
    double servo_gain = -0.94;
    double servo_offset = 0.468;
};

// ============================================================================
// Utility Functions
// ============================================================================
inline double clamp(double value, double lo, double hi) {
    return std::max(lo, std::min(hi, value));
}

/**
 * Map the num_beams sim beam angles to the nearest real lidar beam indices.
 * Only called once — the lidar geometry does not change during a run.
 */
inline std::vector<int> computeBeamIndices(const sensor_msgs::msg::LaserScan& scan,
                                            const Config& config) {
    int n_beams = static_cast<int>(scan.ranges.size());
    std::vector<int> idx(config.num_beams);

    double half = config.lidar_fov_rad / 2.0;
    for (int i = 0; i < config.num_beams; ++i) {
        double sim_angle = -half + (2.0 * half) * i / (config.num_beams - 1);
        int real_idx = static_cast<int>(
            std::round((sim_angle - scan.angle_min) / scan.angle_increment));
        idx[i] = std::min(std::max(real_idx, 0), n_beams - 1);
    }
    return idx;
}

/**
 * Scale policy output [-1,1]^2 to physical steer (rad) + speed (m/s).
 * Mirrors F1TenthRLWrapper._scale_action from training.
 */
inline void scaleAction(const float* action, const Config& config,
                        double& steer, double& speed) {
    steer = clamp(action[0], -1.0, 1.0) * config.max_steer;
    speed = (clamp(action[1], -1.0, 1.0) + 1.0) * 0.5 *
            (config.max_speed - config.min_speed) + config.min_speed;
}

}  // namespace rl_ppo

#endif  // RL_PPO_UTILS_HPP
