#!/usr/bin/env python3
"""
ppo_deploy_node.py
==================
Runs the trained PPO policy on the physical F1TENTH car.
This is the real-car equivalent of what the F1TENTH gym does during training —
it reads sensors, builds the same 110-dim observation, and publishes drive commands.

WHAT THIS NODE DOES (each cycle at 100 Hz):
    1. Read /scan          → 1081 lidar beams from the real Hokuyo
    2. Read /odom          → actual car speed (m/s) from VESC
    3. Read /servo_echo    → actual steering angle (rad) from VESC
    4. Build observation   → resample to 108 beams + normalize → 110-dim vector
                             (identical to F1TenthRLWrapper._process_obs in training)
    5. Run ONNX inference  → 110-dim obs → [steer_cmd, speed_cmd] in [-1, 1]
    6. Scale action        → same math as F1TenthRLWrapper._scale_action
    7. Publish /drive      → AckermannDriveStamped to ackermann_mux

PREREQUISITES:
    1. Export model on PC:
       python3 export_ppo_onnx.py --model models/best/best_model.zip --map my_map5
    2. Copy to car:
       scp deploy/ppo_policy.onnx deploy/ppo_deploy_config.json nvidia@<car-ip>:~/f1tenth_ws/src/rl_ppo/scripts/
    3. Build on car:
       colcon build --packages-select rl_ppo

DRIVING:
    Hold RB to drive — same as every other autonomous mode.
    Release RB or disconnect controller → car stops immediately.

TOPICS:
    Subscribes: /scan, /odom, /sensors/servo_position_command, /joy
    Publishes:  /drive

─────────────────────────────────────────────────────────────────────────────
STUDENT TODOs
─────────────────────────────────────────────────────────────────────────────
This file has 3 TODOs for you to complete:

  TODO 1 — Create subscribers for /scan, /odom, /joy
           These connect the node to the real car's sensor topics.

  TODO 2 — Create a publisher for /drive
           This sends the policy's commands to the car.

  TODO 3 — Complete the _build_obs() observation normalization
           The obs must match exactly what the policy saw during training.

The inference logic (_control_cb, _scale_action) is provided —
once your topics are wired up and obs is correct, the car will drive.
─────────────────────────────────────────────────────────────────────────────
"""

import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy


class PPODeployNode(Node):

    def __init__(self):
        super().__init__('ppo_deploy_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('onnx_path',      '')
        self.declare_parameter('config_path',    '')
        self.declare_parameter('scan_topic',     '/scan')
        self.declare_parameter('odom_topic',     '/odom')
        self.declare_parameter('drive_topic',    '/drive')
        self.declare_parameter('max_speed_cap',  2.0)
        self.declare_parameter('joy_arm_button', 7)   # RB on 8BitDo

        onnx_path   = self.get_parameter('onnx_path').value
        config_path = self.get_parameter('config_path').value
        self.max_speed_cap  = self.get_parameter('max_speed_cap').value
        self.joy_arm_button = self.get_parameter('joy_arm_button').value

        # ── Load config written by export_ppo_onnx.py ────────────────────
        # Contains all scaling constants from the training wrapper —
        # max_speed, min_speed, max_steer, lidar_clip, num_beams, fov
        with open(config_path) as f:
            cfg = json.load(f)

        self.num_beams     = int(cfg['num_beams'])
        self.lidar_clip    = float(cfg['lidar_clip'])
        self.max_speed     = float(cfg['max_speed'])
        self.min_speed     = float(cfg['min_speed'])
        self.max_steer     = float(cfg['max_steer'])
        self.lidar_fov_rad = float(cfg.get('lidar_fov_rad', 4.7))
        control_rate_hz    = float(cfg.get('control_rate_hz', 100.0))

        # ── Load ONNX model ───────────────────────────────────────────────
        import onnxruntime as ort
        self.session = ort.InferenceSession(
            onnx_path, providers=['CPUExecutionProvider'])
        self.onnx_input_name = self.session.get_inputs()[0].name
        self.get_logger().info(
            f'PPO policy loaded | beams={self.num_beams} | '
            f'speed=[{self.min_speed},{self.max_speed}] | '
            f'steer=±{self.max_steer:.4f} | cap={self.max_speed_cap} m/s')

        # ── Beam index lookup (computed once on first scan) ───────────────
        # Maps the 108 sim beam angles to the nearest real lidar beam indices.
        # Only done once — the lidar geometry does not change during a run.
        self.beam_idx = None   # set on first scan callback

        # ── State ─────────────────────────────────────────────────────────
        self.latest_scan = None
        self.speed_meas  = 0.0
        self.steer_echo  = 0.0   # from /sensors/servo_position_command
        self.last_steer  = 0.0
        self.joy_armed   = False

        # ── QoS for sensor topics ─────────────────────────────────────────
        # Sensor topics use BEST_EFFORT — they drop packets rather than queue
        # This keeps latency low (we want the freshest scan, not an old one)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        # ═════════════════════════════════════════════════════════════════
        # ✏️  TODO 1 — Create Subscribers
        # ═════════════════════════════════════════════════════════════════
        # Wire the node up to the car's sensor topics.
        # Each subscriber calls a callback method when a new message arrives.
        #
        # Use self.create_subscription(MsgType, topic, callback, qos)
        #
        # You need to subscribe to 4 topics:
        #
        # (a) Lidar scan — the car's main sensor
        #     Topic:    self.get_parameter('scan_topic').value  → '/scan'
        #     Msg type: LaserScan
        #     Callback: self._scan_cb
        #     QoS:      sensor_qos  (low latency, best effort)
        #
        # (b) Odometry — gives us the car's actual speed
        #     Topic:    self.get_parameter('odom_topic').value  → '/odom'
        #     Msg type: Odometry
        #     Callback: self._odom_cb
        #     QoS:      sensor_qos
        #
        # (c) Servo position — gives us the car's actual steering angle
        #     Topic:    '/sensors/servo_position_command'
        #     Msg type: LaserScan  (the VESC node reuses this msg type)
        #     Callback: self._servo_cb
        #     QoS:      10  (reliable, small queue)
        #
        # (d) Joystick — RB button arms/disarms autonomous driving
        #     Topic:    '/joy'
        #     Msg type: Joy
        #     Callback: self._joy_cb
        #     QoS:      10
        #
        # Reference:
        #   self.create_subscription(MsgType, 'topic_name', self._callback, qos)
        # ─────────────────────────────────────────────────────────────────
        # TODO 1a: Subscribe to /scan
        # self.create_subscription(MsgType, 'topic_name', self._callback, qos)

        # TODO 1b: Subscribe to /odom
        # self.create_subscription(MsgType, 'topic_name', self._callback, qos)

        # TODO 1c: Subscribe to /sensors/servo_position_command
        # self.create_subscription(MsgType, 'topic_name', self._callback, qos)

        # TODO 1d: Subscribe to /joy
        # self.create_subscription(MsgType, 'topic_name', self._callback, qos)
        # ═════════════════════════════════════════════════════════════════

        # ═════════════════════════════════════════════════════════════════
        # ✏️  TODO 2 — Create Drive Publisher
        # ═════════════════════════════════════════════════════════════════
        # Create a publisher that sends drive commands to the car.
        # The ackermann_mux will relay this to the VESC.
        #
        # Use self.create_publisher(MsgType, topic, qos_depth)
        #
        #     Msg type: AckermannDriveStamped
        #     Topic:    self.get_parameter('drive_topic').value  → '/drive'
        #     QoS depth: 10
        #
        # Store it as self.drive_pub so _publish_drive() can use it.
        #
        # Reference:
        #   self.drive_pub = self.create_publisher(MsgType, 'topic_name', depth)
        # ─────────────────────────────────────────────────────────────────
        # TODO 2: create self.drive_pub
        # self.drive_pub = self.create_publisher(MsgType, 'topic_name', depth)
        # ═════════════════════════════════════════════════════════════════

        # ── Control timer — runs _control_cb at 100 Hz ───────────────────
        self.create_timer(1.0 / control_rate_hz, self._control_cb)
        self.get_logger().info(f'Running at {control_rate_hz:.0f} Hz')

    # ─────────────────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg
        if self.beam_idx is None:
            self.beam_idx = self._compute_beam_idx(msg)

    def _odom_cb(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.speed_meas = float(math.sqrt(vx*vx + vy*vy))

    def _servo_cb(self, msg: LaserScan):
        # /sensors/servo_position_command carries servo position [0,1].
        # We invert it back to steering radians using ackermann_to_vesc params.
        # SERVO_GAIN and SERVO_OFFSET must match your ackermann_to_vesc config.
        SERVO_GAIN   = -0.94
        SERVO_OFFSET =  0.468
        if msg.ranges:
            servo_pos = float(msg.ranges[0])
            self.steer_echo = (servo_pos - SERVO_OFFSET) / SERVO_GAIN

    def _joy_cb(self, msg: Joy):
        # RB button — same as every other autonomous mode
        if self.joy_arm_button < len(msg.buttons):
            self.joy_armed = bool(msg.buttons[self.joy_arm_button])

    # ─────────────────────────────────────────────────────────────────────

    def _compute_beam_idx(self, scan: LaserScan) -> np.ndarray:
        """Map sim beam angles to real lidar indices (computed once)."""
        n_beams     = len(scan.ranges)
        real_angles = np.arange(n_beams) * scan.angle_increment + scan.angle_min
        half        = self.lidar_fov_rad / 2.0
        sim_angles  = np.linspace(-half, half, self.num_beams)
        idx         = np.searchsorted(real_angles, sim_angles)
        return np.clip(idx, 0, n_beams - 1).astype(np.int32)

    def _build_obs(self, scan: LaserScan) -> np.ndarray:
        """
        Build the 110-dim observation exactly as in training.
        This must match F1TenthRLWrapper._process_obs() identically —
        if it differs, the policy will see different inputs than it trained on.
        """
        # Sanitize — real lidar returns inf/nan for out-of-range readings
        # The sim never does this, so we must handle it here
        ranges = np.array(scan.ranges, dtype=np.float32)
        ranges = np.where(
            np.isfinite(ranges) & (ranges > float(scan.range_min)),
            ranges, self.lidar_clip)

        # Resample from real lidar angles → sim beam angles
        beams = ranges[self.beam_idx]

        # ═════════════════════════════════════════════════════════════════
        # ✏️  TODO 3 — Normalize the Observation
        # ═════════════════════════════════════════════════════════════════
        # The policy was trained on observations normalized to [0, 1].
        # If you pass raw values here, the policy sees completely different
        # numbers than it trained on and will behave randomly.
        #
        # You need to normalize 3 things — same formulas as training:
        #
        # (a) Lidar beams — clip to max range then divide
        #     Hint: np.clip(beams, 0.0, self.lidar_clip) / self.lidar_clip
        #
        # (b) Speed — divide by max speed, clip to [0, 1]
        #     self.speed_meas is in m/s, self.max_speed is the training limit
        #     Hint: np.clip(self.speed_meas / self.max_speed, 0.0, 1.0)
        #
        # (c) Steering angle — map [−max_steer, +max_steer] → [0, 1]
        #     self.steer_echo is in radians, self.max_steer is the limit
        #     Hint: same formula as _process_obs in the training wrapper:
        #           (steer + max_steer) / (2 * max_steer), clipped to [0, 1]
        #
        # Finally concatenate: [beams_normalized, speed_n, steer_n]
        # Return shape must be (110,) as float32
        # ─────────────────────────────────────────────────────────────────
        # TODO 3a: clip and normalize lidar beams to [0, 1]
        # TODO 3b: normalize speed to [0, 1]
        # TODO 3c: normalize steering angle to [0, 1]

        return np.concatenate([beams, [speed_n, steer_n]]).astype(np.float32)
        # ═════════════════════════════════════════════════════════════════

    def _scale_action(self, action: np.ndarray):
        """Scale policy output [-1,1]^2 to physical steer (rad) + speed (m/s)."""
        steer = float(np.clip(action[0], -1.0, 1.0)) * self.max_steer
        speed = (float(np.clip(action[1], -1.0, 1.0)) + 1.0) * 0.5 * \
                (self.max_speed - self.min_speed) + self.min_speed
        return steer, speed

    def _publish_drive(self, steer: float, speed: float):
        msg = AckermannDriveStamped()
        msg.header.stamp         = self.get_clock().now().to_msg()
        msg.drive.steering_angle = steer
        msg.drive.speed          = speed
        self.drive_pub.publish(msg)

    # ─────────────────────────────────────────────────────────────────────

    def _control_cb(self):
        # Called at 100 Hz — this is the main inference loop.
        # Mirrors the training loop: obs → policy → action → drive command.

        # No scan yet — wait for first message
        if self.latest_scan is None or self.beam_idx is None:
            return

        # RB not held — publish stop so mux sees an active command
        if not self.joy_armed:
            self._publish_drive(0.0, 0.0)
            return

        # Build obs and run inference
        obs    = self._build_obs(self.latest_scan)
        action = self.session.run(
            None, {self.onnx_input_name: obs.reshape(1, -1)})[0][0]

        steer, speed = self._scale_action(action)
        speed = min(speed, self.max_speed_cap)
        self.last_steer = steer

        self._publish_drive(steer, speed)


def main(args=None):
    rclpy.init(args=args)
    node = PPODeployNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node._publish_drive(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
