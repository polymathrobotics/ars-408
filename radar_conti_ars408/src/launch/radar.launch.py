# Copyright 2026 Polymath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value='', description='Full path to the ROS2 parameters file to use for the radar node'
    )
    lifecycle_nodes = ['radar_node']

    autostart = LaunchConfiguration('autostart')
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true', description='Automatically startup the nav2 stack'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='False', description=str('Use sim time argument for whether to force it')
    )

    namespace = LaunchConfiguration('namespace')
    namespace_arg = DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace')

    log_level = LaunchConfiguration('log_level')
    log_level_arg = DeclareLaunchArgument('log_level', default_value='info', description='log level')

    # Nodes launching commands
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='radar_lifecycle_manager',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}, {'autostart': autostart}, {'node_names': lifecycle_nodes}],
    )

    radar_node = Node(
        package='radar_conti_ars408',
        executable='radar_conti_ars408_composition',
        name='radar_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[LaunchConfiguration('params_file')],
    )

    ld = LaunchDescription()
    ld.add_action(log_level_arg)
    ld.add_action(params_file_arg)
    ld.add_action(autostart_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(namespace_arg)
    ld.add_action(radar_node)

    ld.add_action(start_lifecycle_manager_cmd)

    return ld
