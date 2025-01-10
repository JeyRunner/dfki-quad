import os
import sys
import re

import numpy as np
import rclpy
import rclpy.time
from ament_index_python.packages import get_package_share_directory
from interfaces.msg import QuadState
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    if "sim:=go2" in sys.argv[4:] or "sim:=unitree" in sys.argv[4:]:
        sim = True
        unitree = True
        config_file = "mit_controller_sim_go2.yaml"
    elif "real:=go2" in sys.argv[4:] or "real:=unitree" in sys.argv[4:]:
        sim = False
        unitree = True
        config_file = "mit_controller_real_go2.yaml"
    else:
        print("Please specify param 'sim' or 'real' and robot. E.g. 'sim:=ulab' or 'real:=go2'.")
        exit()

    pkg_controllers = get_package_share_directory("controllers")
    controller_config_path = os.path.join(pkg_controllers, "config", config_file)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value=str(sim),
        description="Use simulation clock if true. Default is false.",
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default=str(
        sim))  # This variable is during launch replaced with the respective LaunchArgument declared by DeclareLaunchArgument

    # joystick to goal traj node
    joy_to_target = Node(
        package="controllers",
        name="joy_to_target",
        executable="joy_tgui.py", #"joy_fake_to_target.py",
        parameters=[controller_config_path, {"use_sim_time": use_sim_time}],
        # parameters=[config["js2mpc"][mode]],
        output="screen",
        shell=True,
        #emulate_tty=True
    )

    #joy_to_target = ExecuteProcess(
    #    cmd=['python3', "src/controllers/scripts/joy_tgui.py --ros-args -r __node:=joy_to_target --params-file /root/ros2_ws/install/controllers/share/#controllers/config/mit_controller_sim_go2.yaml"],
    #    output='screen',
    #    shell=True,
    #    emulate_tty=True  # Enables pseudo-terminal support
    #)

    return LaunchDescription([
        joy_to_target,
        declare_use_sim_time_cmd
    ])
