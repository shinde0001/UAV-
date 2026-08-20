import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    px4_dir = '/home/parth/gr/src/PX4-Autopilot'
    world_path = '/home/parth/gr/src/uav_gazebo/worlds/simple_forest.world'
    
    # We must add our custom model path to GAZEBO_MODEL_PATH
    custom_model_dir = '/home/parth/gr/src/uav_description/models'
    env = os.environ.copy()
    if 'GAZEBO_MODEL_PATH' in env:
        env['GAZEBO_MODEL_PATH'] += f':{custom_model_dir}'
    else:
        env['GAZEBO_MODEL_PATH'] = custom_model_dir
        
    env['PX4_SITL_WORLD'] = world_path
    env['HEADLESS'] = '1'
    env['NO_PXH'] = '1'
    env['GAZEBO_MODEL_DATABASE_URI'] = ''

    # Start PX4 SITL and Gazebo
    # PX4's make target will start Gazebo and the SITL instance.
    # We set PX4_SYS_AUTOSTART for Iris (10016) but tell it to use our custom model name
    px4_cmd = [
        'make', 'px4_sitl', 'gazebo'
    ]
    px4_process = ExecuteProcess(
        cmd=px4_cmd,
        cwd=px4_dir,
        output='screen',
        env=env,
        emulate_tty=True
    )
    
    # MAVROS
    # Usually launched via mavros_node, but for SITL we can use px4.launch
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

    # Autonomy Stack Nodes
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
    
    # Delay nodes that require MAVROS and TF
    delayed_nodes = TimerAction(
        period=5.0,
        actions=[localization_node, obstacle_node, planner_node, mission_node]
    )

    return LaunchDescription([
        px4_process,
        mavros_node,
        delayed_nodes
    ])
