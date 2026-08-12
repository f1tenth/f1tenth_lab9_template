#!/usr/bin/env python3
"""
ppo_deploy.launch.py
====================
Launches the trained PPO policy on the physical F1TENTH car.

STEP-BY-STEP (do these in order):

  STEP 1 — On your PC, export the trained model:
      python3 export_ppo_onnx.py --model models/best/best_model.zip --map my_map5
      → produces deploy/ppo_policy.onnx + deploy/ppo_deploy_config.json

  STEP 2 — Copy both files to the car:
      scp deploy/ppo_policy.onnx deploy/ppo_deploy_config.json \\
          nvidia@<car-ip>:~/f1tenth_ws/src/rl_ppo/scripts/

  STEP 3 — Build on the car:
      cd ~/f1tenth_ws
      colcon build --packages-select rl_ppo
      source install/setup.bash

  STEP 4 — Terminal 1: bring up the car stack (lidar + VESC + mux + joy):
      ros2 launch f1tenth_stack bringup_launch.py

  STEP 5 — Terminal 2: launch the PPO policy:
      ros2 launch rl_ppo ppo_deploy.launch.py max_speed_cap:=1.5
      (start slow — raise max_speed_cap once laps are clean)

  DRIVING:
      Hold RB → policy drives.  Release RB → car stops.
      Same as every other autonomous mode — no extra arming needed.

USEFUL VARIANTS:
    ros2 launch rl_ppo ppo_deploy.launch.py max_speed_cap:=4.0
    ros2 launch rl_ppo ppo_deploy.launch.py record:=true
"""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share        = get_package_share_directory('rl_ppo')
    default_onnx = os.path.join(share, 'scripts', 'ppo_policy.onnx')
    default_cfg  = os.path.join(share, 'scripts', 'ppo_deploy_config.json')

    args = [
        # Paths to the two files produced by export_ppo_onnx.py
        DeclareLaunchArgument('onnx_path',    default_value=default_onnx),
        DeclareLaunchArgument('config_path',  default_value=default_cfg),
        # Hard speed ceiling — start at 1.5-2.0, raise only after clean laps
        DeclareLaunchArgument('max_speed_cap', default_value='2.0'),
        # Set record:=true to capture /scan /odom /drive /teleop for analysis
        DeclareLaunchArgument('record',        default_value='false'),
    ]

    ppo_node = Node(
        package='rl_ppo',
        executable='ppo_deploy_node.py',
        name='ppo_deploy_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'onnx_path':    LaunchConfiguration('onnx_path'),
            'config_path':  LaunchConfiguration('config_path'),
            'max_speed_cap': LaunchConfiguration('max_speed_cap'),
        }],
    )

    # Optional bag recording — enabled with record:=true
    # Saves to ~/ros2_bags/ppo_YYYYMMDD_HHMMSS/ for offline analysis
    bag_dir = os.path.expanduser(
        '~/ros2_bags/ppo_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    bag = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record')),
        cmd=['ros2', 'bag', 'record', '-o', bag_dir,
             '/scan', '/odom', '/drive', '/teleop'],
        output='screen',
    )

    return LaunchDescription(args + [ppo_node, bag])
