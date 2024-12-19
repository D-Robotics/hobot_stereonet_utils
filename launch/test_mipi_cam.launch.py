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
    mipi_image_width_arg = DeclareLaunchArgument(
        "mipi_image_width", default_value="1280", description="mipi_image_width"
    )

    mipi_image_height_arg = DeclareLaunchArgument(
        "mipi_image_height", default_value="640", description="mipi_image_height"
    )

    # 零拷贝环境配置
    shared_mem_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hobot_shm"), "launch/hobot_shm.launch.py"
            )
        )
    )

    # mipi双目相机
    dual_mipi_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mipi_cam"),
                "launch/mipi_cam_dual_channel.launch.py",
            )
        ),
        launch_arguments={
            "mipi_image_width": LaunchConfiguration("mipi_image_width"),
            "mipi_image_height": LaunchConfiguration("mipi_image_height"),
            "mipi_frame_ts_type": "realtime",
            "frame_id": "pcl_link",
            "log_level": "error",
        }.items(),
    )

    # 编码节点
    codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hobot_codec"),
                "launch/hobot_codec_encode.launch.py",
            )
        ),
        launch_arguments={
            "codec_in_mode": "ros",
            "codec_out_mode": "ros",
            # 左图和右图拼接后的图
            "codec_sub_topic": "/image_combine_raw",
            "codec_in_format": "nv12",
            "codec_pub_topic": "/image_jpeg",
            "codec_out_format": "jpeg",
            "log_level": "warn",
        }.items(),
    )

    # web展示节点
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("websocket"), "launch/websocket.launch.py"
            )
        ),
        launch_arguments={
            "websocket_image_topic": "/image_jpeg",
            "websocket_only_show_image": "true",
            # 'websocket_smart_topic': '/detect_depth_result'
        }.items(),
    )

    return LaunchDescription(
        [
            mipi_image_width_arg,
            mipi_image_height_arg,
            shared_mem_node,
            dual_mipi_cam,
            codec_node,
            web_node,
        ]
    )
