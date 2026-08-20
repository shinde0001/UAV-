import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    mavros_launch_dir = os.path.join(get_package_share_directory('mavros'), 'launch')
    mavros_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(mavros_launch_dir, 'px4.launch')),
        launch_arguments={
            'fcu_url': 'udp://:14540@127.0.0.1:14557',
            'gcs_url': 'udp://@localhost',
            'tgt_system': '1',
            'tgt_component': '1'
        }.items()
    )

    localization_node = Node(
        package='uav_localization',
        executable='uav_localization_node',
        output='screen'
    )
    
    obstacle_node = Node(
        package='uav_obstacle_detection',
        executable='local_obstacle_filter',
        output='screen'
    )
    
    planner_node = Node(
        package='uav_planner',
        executable='planner_node',
        output='screen'
    )
    
    mission_node = Node(
        package='uav_mission',
        executable='mission_node',
        output='screen'
    )

    return LaunchDescription([
        mavros_node,
        localization_node,
        obstacle_node,
        planner_node,
        mission_node
    ])
