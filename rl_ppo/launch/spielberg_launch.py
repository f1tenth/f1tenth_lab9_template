"""What the autograder runs in the simulator (one lap of Spielberg):

    ros2 launch rl_ppo spielberg_launch.py

Start your own nodes only: the simulator is already running, and the
autograder plays the rest of the car (it holds RB on /joy, republishes the
simulator's odometry on /odom and reports the steering angle on
/sensors/servo_position_command). On your laptop `ros2 run rl_ppo sim_car.py`
does the same, see the README.

ppo_deploy.launch.py stays the launch file for the real car.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# scripts/ppo_deploy_node.py ('ppo_deploy_node' is the C++ node)
EXECUTABLE = 'ppo_deploy_node.py'

# The 2.0 m/s default of max_speed_cap is a safety setting for the real car.
# The simulator lap runs without a cap, so your policy drives at the speeds
# it was trained for. Lower it here if you want a slower lap.
PARAMETERS = {'max_speed_cap': 20.0}


def generate_launch_description():
    scripts = os.path.join(get_package_share_directory('rl_ppo'), 'scripts')
    return LaunchDescription([
        # The two files export_ppo_onnx.py writes, committed in rl_ppo/scripts/
        DeclareLaunchArgument(
            'onnx_path', default_value=os.path.join(scripts, 'ppo_policy.onnx')),
        DeclareLaunchArgument(
            'config_path', default_value=os.path.join(scripts, 'ppo_deploy_config.json')),
        Node(
            package='rl_ppo',
            executable=EXECUTABLE,
            output='screen',
            parameters=[PARAMETERS, {
                'onnx_path': LaunchConfiguration('onnx_path'),
                'config_path': LaunchConfiguration('config_path'),
            }],
        ),
    ])
