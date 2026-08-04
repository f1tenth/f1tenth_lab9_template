#ifndef MPC_UTILS_HPP
#define MPC_UTILS_HPP

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace mpc {

// ============================================================================
// Constants
// ============================================================================
constexpr int NX = 4;  // State size: [x, y, v, yaw]
constexpr int NU = 2;  // Input size: [accel, steer]
constexpr double kPi = 3.14159265358979323846;

// State indices
enum StateIdx { X = 0, Y = 1, V = 2, YAW = 3 };

// Input indices
enum InputIdx { ACCEL = 0, STEER = 1 };

// ============================================================================
// Configuration - Students may tune these values
// ============================================================================
struct Config {
    int NXK = NX;  // State dimension
    int NU = NU;   // Input dimension
    int TK = 8;    // Prediction horizon

    // ---------------------------------------------------
    // TODO: you may need to tune the following matrices
    // Input cost matrix - penalty for inputs [accel, steer]
    Eigen::Matrix2d Rk = (Eigen::Matrix2d() << 0.01, 0.0, 0.0, 100.0).finished();

    // Input difference cost - penalty for change of inputs
    Eigen::Matrix2d Rdk = (Eigen::Matrix2d() << 0.01, 0.0, 0.0, 100.0).finished();

    // State error cost - penalty for [x, y, v, yaw]
    Eigen::Matrix4d Qk = (Eigen::Matrix4d() <<
        13.5, 0.0, 0.0, 0.0,
        0.0, 13.5, 0.0, 0.0,
        0.0, 0.0, 5.5, 0.0,
        0.0, 0.0, 0.0, 13.0).finished();

    // Final state error cost
    Eigen::Matrix4d Qfk = (Eigen::Matrix4d() <<
        13.5, 0.0, 0.0, 0.0,
        0.0, 13.5, 0.0, 0.0,
        0.0, 0.0, 5.5, 0.0,
        0.0, 0.0, 0.0, 13.0).finished();
    // ---------------------------------------------------

    int N_IND_SEARCH = 20;
    double DTK = 0.1;       // Time step [s]
    double dlk = 0.03;      // Distance step [m]
    double LENGTH = 0.58;   // Vehicle length [m]
    double WIDTH = 0.31;    // Vehicle width [m]
    double WB = 0.33;       // Wheelbase [m]
    double MIN_STEER = -0.4189;
    double MAX_STEER = 0.4189;
    double MAX_DSTEER = kPi;  // Max steering rate [rad/s]
    double MAX_SPEED = 6.0;
    double MIN_SPEED = 0.0;
    double MAX_ACCEL = 3.0;
};

// ============================================================================
// State representation
// ============================================================================
struct State {
    double x = 0.0;
    double y = 0.0;
    double delta = 0.0;
    double v = 0.0;
    double yaw = 0.0;
    double yawrate = 0.0;
    double beta = 0.0;
};

// ============================================================================
// Utility Functions
// ============================================================================
inline double clamp(double value, double lo, double hi) {
    return std::max(lo, std::min(hi, value));
}

inline double normalizeAngle(double angle) {
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

inline double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    tf2::Quaternion tfq(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tfq).getRPY(roll, pitch, yaw);
    return yaw;
}

/**
 * Find nearest point on piecewise linear trajectory (vectorized).
 * Returns: (nearest_point, distance, t, segment_index)
 * where t is the interpolation parameter [0,1] along the segment.
 */
inline std::tuple<Eigen::Vector2d, double, double, std::size_t>
nearestPoint(const Eigen::Vector2d& point, const Eigen::MatrixXd& trajectory) {
    // trajectory: (2, N) matrix of [x; y] waypoints
    int n = static_cast<int>(trajectory.cols());
    if (n < 2) {
        return {trajectory.col(0), 0.0, 0.0, 0};
    }

    // Segment vectors: diffs[:, i] = trajectory[:, i+1] - trajectory[:, i]
    Eigen::MatrixXd diffs = trajectory.rightCols(n - 1) - trajectory.leftCols(n - 1);

    // Squared lengths of each segment
    Eigen::VectorXd l2s = diffs.colwise().squaredNorm();

    // Vector from each segment start to the query point
    Eigen::MatrixXd to_point = (-trajectory.leftCols(n - 1)).colwise() + point;

    // Dot product of (point - start) with segment direction for each segment
    Eigen::VectorXd dots = (to_point.array() * diffs.array()).colwise().sum();

    // Projection parameter t, clamped to [0, 1]
    Eigen::VectorXd t = (dots.array() / l2s.array().max(1e-9)).cwiseMax(0.0).cwiseMin(1.0);

    // Projected points on each segment: start + t * diff
    Eigen::MatrixXd projections = trajectory.leftCols(n - 1) + (diffs.array().rowwise() * t.transpose().array()).matrix();

    // Distance from query point to each projection
    Eigen::VectorXd dists = (projections.colwise() - point).colwise().norm();

    // Find minimum
    Eigen::Index min_idx;
    double min_dist = dists.minCoeff(&min_idx);

    Eigen::Vector2d nearest = projections.col(min_idx);
    return {nearest, min_dist, t(min_idx), static_cast<std::size_t>(min_idx)};
}

/**
 * Calculate interpolated reference trajectory using proper linear interpolation.
 *
 * Finds the nearest point on the trajectory, then propagates forward using
 * interpolated velocities to find reference points for the MPC horizon.
 * All state variables are linearly interpolated between waypoints.
 *
 * @param x      Current x position
 * @param y      Current y position
 * @param cx     x positions of waypoints (N,)
 * @param cy     y positions of waypoints (N,)
 * @param cv     velocities at waypoints (N,)
 * @param cyaw   yaw angles at waypoints (N,)
 * @param dt     Time step
 * @param horizon Prediction horizon (returns horizon+1 points)
 * @return ref_traj Interpolated reference trajectory (4, horizon+1)
 *                  where rows are [x, y, v, yaw]
 */
inline Eigen::MatrixXd calcInterpolatedRefTrajectory(
    double x, double y,
    const Eigen::VectorXd& cx,
    const Eigen::VectorXd& cy,
    const Eigen::VectorXd& cv,
    const Eigen::VectorXd& cyaw,
    double dt,
    int horizon)
{
    int ncourse = static_cast<int>(cx.size());
    Eigen::MatrixXd ref_traj = Eigen::MatrixXd::Zero(NX, horizon + 1);

    if (ncourse < 2) {
        return ref_traj;
    }

    // Calculate distance between waypoints (assume uniform spacing)
    double dl = std::sqrt(std::pow(cx(1) - cx(0), 2) + std::pow(cy(1) - cy(0), 2));
    if (dl < 1e-9) dl = 1e-9;

    // Build trajectory matrix for nearestPoint (2 x N)
    Eigen::MatrixXd trajectory(2, ncourse);
    trajectory.row(0) = cx.transpose();
    trajectory.row(1) = cy.transpose();

    // Find nearest point and interpolation parameter t ∈ [0, 1]
    auto [nearest_pt, dist, t_current, ind_current] = nearestPoint(
        Eigen::Vector2d(x, y), trajectory);
    (void)nearest_pt; (void)dist;

    // Build t_list: accumulated "distance" in units of segments
    Eigen::VectorXd t_list = Eigen::VectorXd::Zero(horizon + 1);
    t_list(0) = t_current;

    // Get initial interpolated speed
    int ind_next = (static_cast<int>(ind_current) + 1) % ncourse;
    double current_speed = (1.0 - t_current) * cv(ind_current) + t_current * cv(ind_next);

    for (int i = 1; i <= horizon; ++i) {
        // Distance traveled in dt at current_speed, converted to segment units
        t_list(i) = t_list(i - 1) + (current_speed * dt) / dl;

        // Update speed using new interpolation parameter
        double t_frac = std::fmod(t_list(i), 1.0);
        if (t_frac < 0) t_frac += 1.0;

        int seg_idx = (static_cast<int>(std::floor(t_list(i))) + static_cast<int>(ind_current)) % ncourse;
        if (seg_idx < 0) seg_idx += ncourse;
        int seg_next = (seg_idx + 1) % ncourse;

        current_speed = (1.0 - t_frac) * cv(seg_idx) + t_frac * cv(seg_next);
    }

    // Interpolate all state variables
    for (int i = 0; i <= horizon; ++i) {
        int idx = (static_cast<int>(std::floor(t_list(i))) + static_cast<int>(ind_current)) % ncourse;
        if (idx < 0) idx += ncourse;
        int idx_next = (idx + 1) % ncourse;

        double t_frac = std::fmod(t_list(i), 1.0);
        if (t_frac < 0) t_frac += 1.0;

        ref_traj(X, i) = (1.0 - t_frac) * cx(idx) + t_frac * cx(idx_next);
        ref_traj(Y, i) = (1.0 - t_frac) * cy(idx) + t_frac * cy(idx_next);
        ref_traj(V, i) = (1.0 - t_frac) * cv(idx) + t_frac * cv(idx_next);

        // Handle yaw interpolation (unwrap to avoid discontinuities)
        double yaw0 = cyaw(idx);
        double yaw1 = cyaw(idx_next);
        // Wrap difference to [-pi, pi] using atan2
        double yaw_diff = std::atan2(std::sin(yaw1 - yaw0), std::cos(yaw1 - yaw0));
        ref_traj(YAW, i) = yaw0 + t_frac * yaw_diff;
    }

    return ref_traj;
}

// ============================================================================
// Vehicle Dynamics - Linearized discrete-time bicycle model
// x_{k+1} = A * x_k + B * u_k + C
// ============================================================================
inline void getLinearizedModel(double v, double phi, double delta,
                               const Config& config,
                               Eigen::Matrix4d& A,
                               Eigen::Matrix<double, NX, NU>& B,
                               Eigen::Vector4d& C) {
    // State matrix A
    A.setIdentity();
    A(0, 2) = config.DTK * std::cos(phi);
    A(0, 3) = -config.DTK * v * std::sin(phi);
    A(1, 2) = config.DTK * std::sin(phi);
    A(1, 3) = config.DTK * v * std::cos(phi);
    A(3, 2) = config.DTK * std::tan(delta) / config.WB;

    // Input matrix B
    B.setZero();
    B(2, ACCEL) = config.DTK;
    B(3, STEER) = config.DTK * v / (config.WB * std::pow(std::cos(delta), 2));

    // Affine term C
    C.setZero();
    C(0) = config.DTK * v * std::sin(phi) * phi;
    C(1) = -config.DTK * v * std::cos(phi) * phi;
    C(3) = -config.DTK * v * delta / (config.WB * std::pow(std::cos(delta), 2));
}

/**
 * Update state using nonlinear model (for motion prediction)
 */
inline State updateState(const State& state, double accel, double steer,
                         const Config& config) {
    State next = state;
    double delta = clamp(steer, config.MIN_STEER, config.MAX_STEER);

    next.x = state.x + state.v * std::cos(state.yaw) * config.DTK;
    next.y = state.y + state.v * std::sin(state.yaw) * config.DTK;
    next.yaw = normalizeAngle(state.yaw + (state.v / config.WB) * std::tan(delta) * config.DTK);
    next.v = clamp(state.v + accel * config.DTK, config.MIN_SPEED, config.MAX_SPEED);

    return next;
}

// ============================================================================
// Indexing Helpers - For accessing decision variables in the QP
// Decision variable layout: z = [x_0, x_1, ..., x_T, u_0, u_1, ..., u_{T-1}]
// ============================================================================

/** Index of state component at timestep t */
inline int stateIdx(int t, int component, int horizon) {
    (void)horizon;  // unused but kept for clarity
    return t * NX + component;
}

/** Index of input component at timestep t */
inline int inputIdx(int t, int component, int horizon) {
    return (horizon + 1) * NX + t * NU + component;
}

/** Total number of decision variables */
inline int numVars(int horizon) {
    return (horizon + 1) * NX + horizon * NU;
}

}  // namespace mpc

#endif  // MPC_UTILS_HPP
