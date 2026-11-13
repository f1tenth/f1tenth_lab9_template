#!/usr/bin/env python3
"""sim_car.py: the rest of the car, for the simulator.

ppo_deploy_node is written for the real car: it reads /odom, the servo
position the VESC reports and /joy (it drives only while RB is held). The
f1tenth_gym_ros simulator publishes none of these. Run this beside it and
your node runs as on the car:

    ros2 run rl_ppo sim_car.py
    ros2 launch rl_ppo spielberg_launch.py

  /joy                             button 7 (RB) held, 20 Hz
  /odom                            every /ego_racecar/odom message, republished
  /sensors/servo_position_command  the steering angle the simulated car has, as the
                                   servo position the VESC would report (SERVO_GAIN x
                                   angle + SERVO_OFFSET, the values in
                                   ppo_deploy_node.py as handed out), at 100 Hz, in the
                                   type your node subscribed with

The angle is the one your policy saw in training: the simulator turns the
wheels toward the commanded angle at 3.2 rad/s at most, so this node keeps
its own angle that follows your /drive steering command at that rate.

The autograder runs the same code beside the simulator for your lap, so
you do not start this from your launch file.
"""
import argparse
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import Float32, Float64

NAME = 'sim_car'
SERVO_TOPIC = '/sensors/servo_position_command'
MSG = {'sensor_msgs/msg/LaserScan': LaserScan, 'std_msgs/msg/Float64': Float64,
       'std_msgs/msg/Float32': Float32}
GRADER = ('topic_guard_', 'lap_referee', 'gym_bridge', NAME, '_')


class CarShim(Node):
    def __init__(self, args):
        super().__init__(NAME)
        self.args = args
        lenient = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_pub.publish, lenient)
        self.create_subscription(AckermannDriveStamped, '/drive', self.on_drive, lenient)
        self.command = 0.0      # the last /drive steering command
        self.angle = 0.0        # the steering angle the simulated car has
        self.last_tick = None
        self.servo_type = None
        self.servo_pub = None
        self.create_timer(1.0 / args.joy_rate, self.pub_joy)
        self.create_timer(1.0 / args.servo_rate, self.pub_servo)
        self.create_timer(0.25, self.find_servo_type)

    def on_drive(self, msg):
        if msg.header.frame_id == 'lap_referee':
            return
        self.command = float(msg.drive.steering_angle)

    def pub_joy(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [0.0] * 8
        msg.buttons = [0] * 12
        msg.buttons[self.args.arm_button] = 1
        self.joy_pub.publish(msg)

    def find_servo_type(self):
        if self.servo_pub is not None or self.args.no_servo:
            return
        for info in self.get_subscriptions_info_by_topic(SERVO_TOPIC):
            if info.node_name.startswith(GRADER) or info.topic_type not in MSG:
                continue
            self.servo_type = info.topic_type
            self.servo_pub = self.create_publisher(MSG[self.servo_type], SERVO_TOPIC, 10)
            self.get_logger().info(f'servo position published as {self.servo_type}, steering '
                                   + (f'slewed at {self.args.steer_rate:g} rad/s'
                                      if self.args.steer_rate > 0 else 'as commanded'))
            return

    def step_angle(self):
        """Move the angle toward the command at steer_rate over the real time
        since the last tick, clipped to +-max_steer."""
        now = time.monotonic()
        dt = 0.0 if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        lim = self.args.max_steer
        target = min(max(self.command, -lim), lim)
        if self.args.steer_rate <= 0:
            self.angle = target
            return
        step = self.args.steer_rate * dt
        self.angle += min(max(target - self.angle, -step), step)
        self.angle = min(max(self.angle, -lim), lim)

    def pub_servo(self):
        self.step_angle()
        if self.servo_pub is None:
            return
        pos = self.args.servo_gain * self.angle + self.args.servo_offset
        msg = MSG[self.servo_type]()
        if self.servo_type.endswith('LaserScan'):
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.ranges = [pos]
        else:
            msg.data = pos
        self.servo_pub.publish(msg)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--joy-rate', type=float, default=20.0)
    ap.add_argument('--servo-rate', type=float, default=100.0, help='Hz, servo position ticks')
    ap.add_argument('--steer-rate', type=float, default=3.2,
                    help="rad/s the steering angle follows the command (the sim's limit); 0: as commanded")
    ap.add_argument('--max-steer', type=float, default=0.4189, help='rad, the steering limit')
    ap.add_argument('--servo-gain', type=float, default=-0.94)
    ap.add_argument('--servo-offset', type=float, default=0.468)
    ap.add_argument('--arm-button', type=int, default=7)
    ap.add_argument('--no-servo', action='store_true',
                    help='publish no servo position (config.yaml spielberg.servo_echo: false)')
    args = ap.parse_args()
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    rclpy.init()
    node = CarShim(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    except Exception as exc:   # rclpy's ExternalShutdownException on SIGINT
        if 'Shutdown' not in type(exc).__name__:
            raise
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
