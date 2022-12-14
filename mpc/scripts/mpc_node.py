#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

import cvxpy
import math
# TODO CHECK: include needed ROS msg type headers and libraries

class MPC(Node):
    """ 
    Implement Kinematic MPC on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('mpc_node')
        # TODO: create ROS subscribers and publishers

        # TODO: initialize MPC problem

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
