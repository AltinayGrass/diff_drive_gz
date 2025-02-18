# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


from launch_ros.actions import SetRemap


def generate_launch_description():

    # Configure ROS nodes for launch
    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ros_gz_example_bringup')
       
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    use_collision_monitor = LaunchConfiguration('col', default='False')
    
    # Navigation

    nav2_params_file = 'nav2_params.yaml'
    nav2_params = os.path.join(pkg_project_bringup, 'config', nav2_params_file)
    nav2_map = os.path.join(pkg_project_bringup, 'config', 'my_map.yaml')
    if 'nav2_bringup' in get_packages_with_prefixes():
        nav2_action = GroupAction([
            SetRemap('/global_costmap/scan',
                 '/scan'),
            SetRemap('/local_costmap/scan',
                 '/scan'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('ros_gz_example_bringup'), 'launch', 'navigation_launch.py')),
                    launch_arguments=[
                    ('params_file', nav2_params),
                    ('use_sim_time', use_sim_time),
                    ]
            )
        ])

    collision_params_file = 'collision_monitor.yaml'
    collision_params = os.path.join(pkg_project_bringup, 'config', collision_params_file)
    if 'nav2_collision_monitor' in get_packages_with_prefixes():
        collision_action = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('nav2_collision_monitor'), 'launch', 'collision_monitor_node.launch.py')),
                        launch_arguments=[
                        ('params_file', collision_params),
                        ('use_sim_time', use_sim_time),
                        ],
                        condition=IfCondition(use_collision_monitor)
        )
    
    return LaunchDescription([
        collision_action,
        nav2_action,
    ])
