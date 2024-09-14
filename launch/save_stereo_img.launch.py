# Copyright (c) 2024，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    save_num_arg = DeclareLaunchArgument(
        "save_num", default_value="1", description="save num"
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    hobot_stereonet_utils_node = Node(
        package="hobot_stereonet_utils",
        executable="hobot_stereonet_utils",
        output="screen",
        parameters=[{"save_num": LaunchConfiguration("save_num")}],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription(
        [
            save_num_arg,
            log_level_arg,
            hobot_stereonet_utils_node,
        ]
    )

