#!/usr/bin/env python3
import math
from dataclasses import dataclass, field

import cvxpy
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.linalg import block_diag
from scipy.sparse import block_diag, csc_matrix, diags
from sensor_msgs.msg import LaserScan
from utils import nearest_point

# TODO CHECK: include needed ROS msg type headers and libraries

# dataclasses definitions
@dataclass
class mpc_config:
    NXK: int = 4  # length of kinematic state vector: z = [x, y, v, yaw]
    NU: int = 2  # length of input vector: u = = [steering speed, acceleration]
    TK: int = 8  # finite time horizon length kinematic
    Rk: list = field(
        default_factory=lambda: np.diag([0.01, 100.0])
    )  # input cost matrix, penalty for inputs - [accel, steering_speed]
    Rdk: list = field(
        default_factory=lambda: np.diag([0.01, 100.0])
    )  # input difference cost matrix, penalty for change of inputs - [accel, steering_speed]
    Qk: list = field(
        default_factory=lambda: np.diag([13.5, 13.5, 5.5, 13.0])
    )  # state error cost matrix, for the the next (T) prediction time steps [x, y, delta, v, yaw, yaw-rate, beta]
    Qfk: list = field(
        default_factory=lambda: np.diag([13.5, 13.5, 5.5, 13.0])
    )  # final state error matrix, penalty  for the final state constraints: [x, y, delta, v, yaw, yaw-rate, beta]
    N_IND_SEARCH: int = 20  # Search index number
    DTK: float = 0.1  # time step [s] kinematic
    dlk: float = 0.03  # dist step [m] kinematic
    LENGTH: float = 0.58  # Length of the vehicle [m]
    WIDTH: float = 0.31  # Width of the vehicle [m]
    WB: float = 0.33  # Wheelbase [m]
    MIN_STEER: float = -0.4189  # maximum steering angle [rad]
    MAX_STEER: float = 0.4189  # maximum steering angle [rad]
    MAX_DSTEER: float = np.deg2rad(180.0)  # maximum steering speed [rad/s]
    MAX_STEER_V: float = 3.2  # maximum steering speed [rad/s]
    MAX_SPEED: float = 6.0  # maximum speed [m/s]
    MIN_SPEED: float = 0.0  # minimum backward speed [m/s]
    MAX_ACCEL: float = 3.0  # maximum acceleration [m/ss]


@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    delta: float = 0.0
    v: float = 0.0
    yaw: float = 0.0
    yawrate: float = 0.0
    beta: float = 0.0

class MPC(Node):
    """ 
    Implement Kinematic MPC on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('mpc_node')
        # TODO: create ROS subscribers and publishers

        # initialize MPC problem
        self.mpc_prob_init()

    def pose_callback(self, pose_msg):
        pass
        # TODO: Calculate the next reference trajectory for the next T steps
        #       with current vehicle pose

        # TODO: call motion prediction fuction for the next T steps

        # TODO: solve current iteration of optimization with reference trajectory

        # TODO: publish drive message, don't forget to limit the steering angle.

    def calc_ref_trajectory(self, state, waypoints):
        pass

    def predict_motion(self, state_init, accel, steer):
        pass

    def update_state(self, state, accel, steer):
        pass

    def get_model_matrix(self, v, phi, delta):
        pass

    def mpc_prob_init(self):
        pass

    def mpc_prob_solve(self, ref_traj, path_predict, state_init):
        pass

    def linear_mpc_control(self, ref_traj, state_init, accel, steer):
        pass

def main(args=None):
    rclpy.init(args=args)
    print("MPC Initialized")
    mpc_node = MPC()
    rclpy.spin(mpc_node)

    mpc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
