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
import xacro

from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution


from launch_ros.actions import Node


def generate_launch_description():

    # Configure ROS nodes for launch
     
    # gz to ros bridge parameters
    GZ_TO_ROS_LASERSCAN = '@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'
    GZ_TO_ROS_TWIST = '@geometry_msgs/msg/Twist[ignition.msgs.Twist'
    ROS_TO_GZ_TWIST = '@geometry_msgs/msg/Twist]ignition.msgs.Twist'
    GZ_TO_ROS_TF = '@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ros_gz_example_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gz_example_gazebo')
    pkg_project_description = get_package_share_directory('ros_gz_example_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_slam_toolbox = LaunchConfiguration('slam_toolbox', default='False')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    use_nav = LaunchConfiguration('nav', default='False')

    # Load the SDF file from "description" package
    sdf_file  =  os.path.join(pkg_project_description, 'models', 'diff_drive', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Parse robot description from xacro
    robot_description_file = os.path.join(pkg_project_description, 'urdf' , 'motor_drive_gz.urdf')
    
    robot_description_config = xacro.process_file(
        robot_description_file
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    world = PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'warehouse.sdf'
        ])
    
    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [ world , ' -r -v4']}.items(),
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    control_params_file = PathJoinSubstitution(
        [pkg_project_bringup, 'config', 'controllers.yaml'])

    diffdrive_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[control_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['diffdrive_controller', '-c', 'controller_manager'],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['joint_state_broadcaster', '-c', 'controller_manager'],
        output='screen',
    )

    # Ensure diffdrive_controller_node starts after joint_state_broadcaster_spawner
    diffdrive_controller_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diffdrive_controller_node],
        )
    )

    # Navigation
    toolbox_params = os.path.join(pkg_project_bringup, 'config', 'slam_toolbox_params.yaml')
    slam_toolbox = Node(
        parameters=[toolbox_params,
                    {'use_sim_time':  use_sim_time}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(use_slam_toolbox)
    )

    localization_params = PathJoinSubstitution([pkg_project_bringup, 'config', 'localization.yaml'])
    nav2_params_file = 'nav2_params.yaml'
    nav2_params = os.path.join(pkg_project_bringup, 'config', nav2_params_file)
    nav2_map = os.path.join(pkg_project_bringup, 'config', 'my_map.yaml')
    if 'nav2_bringup' in get_packages_with_prefixes():
        localization_action=IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')),
            launch_arguments=[
                ('map', nav2_map),
                ('params_file', localization_params),
                ('use_sim_time', use_sim_time),
            ],
            condition=IfCondition(use_nav))
        nav2_action=IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
            launch_arguments=[
                ('map', nav2_map),
                ('params_file', nav2_params),
                ('use_sim_time', use_sim_time),
            ],
            condition=IfCondition(use_nav))
        
    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'diff_drive.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_example_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )


    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'diff_drive',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.015'
        ],
        output='screen',
        )
    
    # Static transform from <namespace>/odom to odom
    # See https://github.com/ros-controls/ros2_controllers/pull/533
    tf_namespaced_odom_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_namespaced_odom_publisher',
        arguments=['0', '0', '0',
                   '0', '0', '0',
                   'odom','/odom'],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        output='screen',
        #condition=LaunchConfigurationNotEquals('namespace', '')
    )

    # Static transform from <namespace>/base_link to base_link
    tf_namespaced_base_link_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_namespaced_base_link_publisher',
        arguments=['0', '0', '0',
                   '0', '0', '0',
                   'warehouse/diff_drive/base_footprint', 'map/odom/base_footprint'],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        output='screen',
        #condition=LaunchConfigurationNotEquals('namespace', '')
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
    )
    
    laser_scan_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='laserscan_bridge',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                 '/scan' + GZ_TO_ROS_LASERSCAN            ],
            remappings=[
                ('/scan' , 'diff_drive/scan')
            ])

    cmd_vel_bridge_arg = 'diff_drive' + '/cmd_vel' + GZ_TO_ROS_TWIST
    cmd_vel_bridge_remap = ('diff_drive' + '/cmd_vel', 'cmd_vel')

    cmd_vel_robot_bridge_arg = '/model/' + 'diff_drive' + '/cmd_vel' + ROS_TO_GZ_TWIST
    cmd_vel_robot_bridge_remap = (
        '/model/' + 'diff_drive' + '/cmd_vel',
        'diff_drive/cmd_vel_unstamped'
        )
    cmd_vel_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                # cmd_vel_bridge_arg,
                cmd_vel_robot_bridge_arg
            ],
            remappings=[
                # cmd_vel_bridge_remap,
                cmd_vel_robot_bridge_remap
            ])
    
    # odom to base_link tf bridge
    odom_base_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='odom_base_tf_bridge',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '/model/' + 'diff_drive' + '/pose' + GZ_TO_ROS_TF
            ],
            remappings=[
                ('/model/' + 'diff_drive' + '/pose', 'tf')
            ])
    
    # Clock bridge
    clock_bridge = Node(package='ros_gz_bridge',
                        executable='parameter_bridge',
                        name='clock_bridge',
                        output='screen',
                        arguments=[
                          '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
                        ])   
    
    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Open RViz.'),
        DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([pkg_project_bringup, 'config', 'my_map.yaml']),
        description='Full path to map yaml file to load'),
        declare_x_position_cmd,
        declare_y_position_cmd,
        spawn_entity,
        # bridge,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        diffdrive_controller_callback,
        cmd_vel_node,
        # odom_base_node,
        laser_scan_node,
        # footprint_publisher,
        # tf_namespaced_odom_publisher,
        # tf_namespaced_base_link_publisher,
        slam_toolbox,
        localization_action,
        nav2_action,
        rviz,
        clock_bridge,
    ])
