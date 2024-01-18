# Copyright 2023 Clearpath Robotics, Inc.
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
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('world', default_value='empty',
                          description='Ignition World')
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():
    # Directories
    pkg_upc_mrn = get_package_share_directory(
        'upc_mrn')

    # Paths
    sim = PathJoinSubstitution(
        [pkg_upc_mrn, 'launch', 'sim.launch.py'])
    slam = PathJoinSubstitution(
        [pkg_upc_mrn, 'launch', 'slam.launch.py'])
    nav = PathJoinSubstitution(
        [pkg_upc_mrn, 'launch', 'nav.launch.py'])

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([sim]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('namespace', LaunchConfiguration('namespace')),
            ('world', LaunchConfiguration('world')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw'))
        ]
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('namespace', LaunchConfiguration('namespace'))]
    )

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('namespace', LaunchConfiguration('namespace'))]
    )
    
    frontiers = Node(
            package='upc_mrn',
            executable='find_frontiers',
            output='screen'
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(sim)
    ld.add_action(slam)
    ld.add_action(nav)
    ld.add_action(frontiers)
    return ld
