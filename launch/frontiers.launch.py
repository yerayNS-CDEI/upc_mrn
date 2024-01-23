from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

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
        PythonLaunchDescriptionSource(sim),
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
        PythonLaunchDescriptionSource(slam),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('namespace', LaunchConfiguration('namespace'))]
    )

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav)
    )
    
    frontiers = Node(
            package='upc_mrn',
            executable='find_frontiers',
            output='screen'
    )

    # Create launch description and add actions
    ld = LaunchDescription()
    ld.add_action(sim)
    ld.add_action(slam)
    ld.add_action(nav)
    ld.add_action(frontiers)
    return ld
