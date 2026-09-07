import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # 1) ZED wrapper (include its own launch file, in sim mode)
    zed_wrapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch', 'zed_camera.launch.py'
            )
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'sim_mode': 'true',
            'sim_address': '127.0.0.1',
            'sim_port': '30000',
            'use_sim_time' : 'true',
        }.items(),
    )

    # 2) Swerve controller
    controller = Node(
        package='athena_controller',
        executable='swerve_controller',   # <-- match your entry-point name
        name='swerve_controller',
        output='screen',
    )

    # 3) Static transform: base_link -> zed_camera_link (camera mount)
    zed_mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_mount_tf',
        arguments=[
            '--x', '0.1', '--y', '0.0', '--z', '0.15',   # <-- your ZED offset
            '--frame-id', 'base_link',
            '--child-frame-id', 'zed_camera_link',
        ],
    )

    # 4) RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time' : True}],
        # optional: auto-load a saved config so you don't re-add displays:
        # arguments=['-d', os.path.join(
        #     get_package_share_directory('athena_controller'),
        #     'config', 'rover.rviz')],
    )

    return LaunchDescription([
        zed_wrapper,
        controller,
        zed_mount_tf,
        rviz,
    ])