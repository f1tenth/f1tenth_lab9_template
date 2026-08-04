/**
 * MPC Node for F1Tenth
 *
 * Implement Kinematic MPC on the car.
 * This is just a template, you are free to implement your own node!
 */

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "mpc/mpc_utils.hpp"

using namespace mpc;

class MPC : public rclcpp::Node {
public:
    MPC() : Node("mpc_node") {
        // TODO: create ROS subscribers and publishers
        //       use the MPC as a tracker (similar to pure pursuit)

        // TODO: get waypoints here
        waypoints_ = Eigen::MatrixXd();  // Should be (4 x N) matrix: [x; y; yaw; v]

        config_ = Config();
        odelta_v_ = std::vector<double>();
        oa_ = std::vector<double>();

        // Initialize MPC problem
        mpc_prob_init();

        RCLCPP_INFO(get_logger(), "MPC Node initialized");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
        (void)odom_msg;  // Suppress unused warning

        // TODO: extract pose from ROS msg
        State vehicle_state;

        // TODO: Calculate the next reference trajectory for the next T steps
        //       with current vehicle pose.
        //       ref_x, ref_y, ref_yaw, ref_v are columns of waypoints_
        Eigen::MatrixXd ref_path;  // = calc_ref_trajectory(vehicle_state, ...)

        Eigen::Vector4d x0;
        x0 << vehicle_state.x, vehicle_state.y, vehicle_state.v, vehicle_state.yaw;

        // TODO: solve the MPC control problem
        Eigen::VectorXd ox, oy, ov, oyaw;
        Eigen::MatrixXd state_predict;
        // std::tie(oa_, odelta_v_, ox, oy, oyaw, ov, state_predict) =
        //     linear_mpc_control(ref_path, x0, oa_, odelta_v_);

        // TODO: publish drive message
        double steer_output = 0.0;  // = odelta_v_[0];
        double speed_output = 0.0;  // = vehicle_state.v + oa_[0] * config_.DTK;
        (void)steer_output;
        (void)speed_output;
    }

    /**
     * Create MPC quadratic optimization problem using OSQP-Eigen
     * Will be solved every iteration for control.
     *
     * More MPC problem information here: https://osqp.org/docs/examples/mpc.html
     *
     * The QP has the form:
     *   min  (1/2) z'Pz + q'z
     *   s.t. l <= Az <= u
     *
     * Decision variables: z = [x_0, x_1, ..., x_T, u_0, u_1, ..., u_{T-1}]
     * where x_t = [x, y, v, yaw] and u_t = [accel, steer]
     */
    void mpc_prob_init() {
        int n_vars = numVars(config_.TK);

        // Initialize decision variable vectors for warm starting
        // Vehicle State: x = [x, y, v, yaw] for T+1 timesteps
        xk_ = Eigen::MatrixXd::Zero(config_.NXK, config_.TK + 1);
        // Control Input: u = [accel, steer] for T timesteps
        uk_ = Eigen::MatrixXd::Zero(config_.NU, config_.TK);

        // Reference trajectory parameter (to be updated each solve)
        ref_traj_k_ = Eigen::MatrixXd::Zero(config_.NXK, config_.TK + 1);

        // Initial state parameter (to be updated each solve)
        x0k_ = Eigen::Vector4d::Zero();

        // Initialize linearized dynamics matrices (will be updated each solve)
        // These form block diagonal matrices for the full horizon
        A_block_.resize(config_.TK);
        B_block_.resize(config_.TK);
        C_block_.resize(config_.TK);
        for (int t = 0; t < config_.TK; ++t) {
            A_block_[t] = Eigen::Matrix4d::Identity();
            B_block_[t] = Eigen::Matrix<double, NX, NU>::Zero();
            C_block_[t] = Eigen::Vector4d::Zero();
        }

        // --------------------------------------------------------
        // Build the Hessian matrix P (quadratic cost)
        // This encodes the objective function and stays constant
        // --------------------------------------------------------

        // TODO: Objective part 1: Influence of the control inputs
        //       Inputs u multiplied by the penalty Rk
        //       This adds u_t' * Rk * u_t for each timestep

        // TODO: Objective part 2: Deviation of the vehicle from reference trajectory
        //       States weighted by Qk, final timestep T weighted by Qfk
        //       This adds (x_t - x_ref)' * Qk * (x_t - x_ref)

        // TODO: Objective part 3: Difference from one control input to the next
        //       Weighted by Rdk to penalize jerky inputs
        //       This adds (u_{t+1} - u_t)' * Rdk * (u_{t+1} - u_t)

        // Build Hessian from the above objectives
        std::vector<Eigen::Triplet<double>> P_triplets;
        // ... add triplets for objectives above ...
        P_.resize(n_vars, n_vars);
        P_.setFromTriplets(P_triplets.begin(), P_triplets.end());

        // --------------------------------------------------------
        // Build the constraint matrix structure A
        // Constraints: dynamics, bounds, rate limits
        // --------------------------------------------------------

        // Count total constraints to size the matrix
        int n_constraints = 0;
        // Dynamics: NX constraints per timestep, TK timesteps
        n_constraints += config_.NXK * config_.TK;
        // Initial state: NX constraints
        n_constraints += config_.NXK;
        // TODO: add counts for your bound constraints

        // TODO: Constraint part 1: Dynamics constraints
        //       x_{t+1} = A_t * x_t + B_t * u_t + C_t
        //       Rearranged: x_{t+1} - A_t * x_t - B_t * u_t = C_t

        // TODO: Constraint part 2: Steering rate constraints
        //       |u_{t+1}[STEER] - u_t[STEER]| <= MAX_DSTEER * DTK

        // TODO: Constraint part 3: State and input bounds
        //       MIN_SPEED <= v <= MAX_SPEED
        //       -MAX_ACCEL <= accel <= MAX_ACCEL
        //       MIN_STEER <= steer <= MAX_STEER

        // TODO: Initial state constraint
        //       x_0 = x0k_

        std::vector<Eigen::Triplet<double>> A_triplets;
        // ... add triplets for constraints above ...
        A_.resize(n_constraints, n_vars);
        A_.setFromTriplets(A_triplets.begin(), A_triplets.end());

        // Initialize bound vectors
        l_ = Eigen::VectorXd::Zero(n_constraints);
        u_ = Eigen::VectorXd::Zero(n_constraints);

        // --------------------------------------------------------
        // Initialize the OSQP solver
        // --------------------------------------------------------
        solver_.settings()->setVerbosity(false);
        solver_.settings()->setWarmStart(true);
        solver_.settings()->setPolish(true);

        solver_.data()->setNumberOfVariables(n_vars);
        solver_.data()->setNumberOfConstraints(n_constraints);
        solver_.data()->setHessianMatrix(P_);
        Eigen::VectorXd initial_grad = Eigen::VectorXd::Zero(n_vars);
        solver_.data()->setGradient(initial_grad);
        solver_.data()->setLinearConstraintsMatrix(A_);
        solver_.data()->setLowerBound(l_);
        solver_.data()->setUpperBound(u_);

        solver_.initSolver();
    }

    /**
     * Calculate interpolated reference trajectory for T steps.
     *
     * Uses proper linear interpolation between waypoints based on velocity
     * propagation, rather than snapping to discrete waypoint indices.
     */
    Eigen::MatrixXd calc_ref_trajectory(const State& state,
                                        const Eigen::VectorXd& cx,
                                        const Eigen::VectorXd& cy,
                                        const Eigen::VectorXd& cyaw,
                                        const Eigen::VectorXd& sp) {
        return calcInterpolatedRefTrajectory(
            state.x, state.y,
            cx, cy, sp, cyaw,
            config_.DTK,
            config_.TK
        );
    }

    /**
     * Predict vehicle motion for T steps using nonlinear model
     */
    Eigen::MatrixXd predict_motion(const Eigen::Vector4d& x0,
                                   const std::vector<double>& oa,
                                   const std::vector<double>& od) {
        Eigen::MatrixXd path_predict = Eigen::MatrixXd::Zero(config_.NXK, config_.TK + 1);
        path_predict.col(0) = x0;

        State state;
        state.x = x0(X);
        state.y = x0(Y);
        state.v = x0(V);
        state.yaw = x0(YAW);

        for (int t = 1; t <= config_.TK; ++t) {
            state = updateState(state, oa[t-1], od[t-1], config_);
            path_predict(X, t) = state.x;
            path_predict(Y, t) = state.y;
            path_predict(V, t) = state.v;
            path_predict(YAW, t) = state.yaw;
        }

        return path_predict;
    }

    /**
     * Solve the MPC optimization problem
     */
    std::tuple<bool, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd,
               Eigen::VectorXd, Eigen::VectorXd>
    mpc_prob_solve(const Eigen::MatrixXd& ref_traj,
                   const Eigen::MatrixXd& path_predict,
                   const Eigen::Vector4d& x0) {
        // Update initial state
        x0k_ = x0;

        // Update linearized dynamics matrices along predicted path
        for (int t = 0; t < config_.TK; ++t) {
            getLinearizedModel(path_predict(V, t), path_predict(YAW, t), 0.0,
                              config_, A_block_[t], B_block_[t], C_block_[t]);
        }

        // TODO: Update the constraint matrix A_ with new dynamics matrices
        //       Use solver_.updateLinearConstraintsMatrix(A_);

        // TODO: Update the bounds l_ and u_ with:
        //       - C vectors from dynamics
        //       - Initial state x0
        //       Use solver_.updateBounds(l_, u_);

        // Update gradient vector q (depends on reference trajectory)
        Eigen::VectorXd q = Eigen::VectorXd::Zero(numVars(config_.TK));
        // TODO: q = -2 * Q * x_ref for state tracking objective
        solver_.updateGradient(q);

        // Solve
        if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
            return {false, {}, {}, {}, {}, {}};
        }

        Eigen::VectorXd solution = solver_.getSolution();

        // Extract solution
        Eigen::VectorXd ox(config_.TK + 1), oy(config_.TK + 1);
        Eigen::VectorXd ov(config_.TK + 1), oyaw(config_.TK + 1);
        Eigen::VectorXd oa(config_.TK), odelta(config_.TK);

        for (int t = 0; t <= config_.TK; ++t) {
            ox(t) = solution(stateIdx(t, X, config_.TK));
            oy(t) = solution(stateIdx(t, Y, config_.TK));
            ov(t) = solution(stateIdx(t, V, config_.TK));
            oyaw(t) = solution(stateIdx(t, YAW, config_.TK));
        }
        for (int t = 0; t < config_.TK; ++t) {
            oa(t) = solution(inputIdx(t, ACCEL, config_.TK));
            odelta(t) = solution(inputIdx(t, STEER, config_.TK));
        }

        return {true, oa, odelta, ox, oy, oyaw};
    }

    /**
     * MPC control with updating operational point iteratively
     */
    std::tuple<std::vector<double>, std::vector<double>,
               Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd,
               Eigen::MatrixXd>
    linear_mpc_control(const Eigen::MatrixXd& ref_path,
                       const Eigen::Vector4d& x0,
                       std::vector<double> oa,
                       std::vector<double> od) {
        if (oa.empty()) oa.assign(config_.TK, 0.0);
        if (od.empty()) od.assign(config_.TK, 0.0);

        // Predict motion for linearization points
        Eigen::MatrixXd path_predict = predict_motion(x0, oa, od);

        // Solve MPC
        auto [success, mpc_a, mpc_delta, mpc_x, mpc_y, mpc_yaw] =
            mpc_prob_solve(ref_path, path_predict, x0);

        if (!success) {
            return {{}, {}, {}, {}, {}, {}, path_predict};
        }

        std::vector<double> oa_out(mpc_a.data(), mpc_a.data() + mpc_a.size());
        std::vector<double> od_out(mpc_delta.data(), mpc_delta.data() + mpc_delta.size());

        return {oa_out, od_out, mpc_x, mpc_y, mpc_yaw,
                Eigen::VectorXd(), path_predict};
    }

    // Configuration
    Config config_;

    // Waypoints matrix: [x; y; yaw; v] as columns
    Eigen::MatrixXd waypoints_;

    // Previous control inputs for warm starting
    std::vector<double> odelta_v_;
    std::vector<double> oa_;

    // MPC problem variables
    Eigen::MatrixXd xk_;        // State trajectory
    Eigen::MatrixXd uk_;        // Input trajectory
    Eigen::MatrixXd ref_traj_k_;  // Reference trajectory
    Eigen::Vector4d x0k_;       // Initial state

    // Linearized dynamics matrices
    std::vector<Eigen::Matrix4d> A_block_;
    std::vector<Eigen::Matrix<double, NX, NU>> B_block_;
    std::vector<Eigen::Vector4d> C_block_;

    // QP matrices (sparse)
    Eigen::SparseMatrix<double> P_;  // Hessian
    Eigen::SparseMatrix<double> A_;  // Constraint matrix
    Eigen::VectorXd l_, u_;          // Constraint bounds

    // OSQP solver (initialized once, updated each iteration)
    OsqpEigen::Solver solver_;

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPC>());
    rclcpp::shutdown();
    return 0;
}
