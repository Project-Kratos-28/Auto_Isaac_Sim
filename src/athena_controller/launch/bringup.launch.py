from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # The ZED stack is optional: zed_wrapper only builds once the ZED SDK is
    # installed. Launch with use_zed:=true to bring the camera up as well.
    use_zed = LaunchConfiguration('use_zed')

    declare_use_zed = DeclareLaunchArgument(
        'use_zed',
        default_value='false',
        description='Start the ZED wrapper and its camera-mount transform',
    )

    # 1) ZED wrapper (include its own launch file, in sim mode)
    zed_wrapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'
            ])
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'sim_mode': 'true',
            'sim_address': '127.0.0.1',
            'sim_port': '30000',
            'use_sim_time' : 'true',
        }.items(),
        condition=IfCondition(use_zed),
    )

    # 2) Swerve controller
    controller = Node(
        package='athena_controller',
        executable='swerve_controller',   # <-- match your entry-point name
        name='swerve_controller',
        output='screen',
    )

    # 3) Static transform: base_link -> zed_camera_link (camera mount)
    #
    # Published unconditionally. This describes where the camera is mounted on
    # the chassis, which is true whether or not the ZED node is running, and the
    # ZED wrapper's own URDF publishes zed_camera_link as an orphan root. Without
    # this the ZED frames are disconnected from base_link, so nothing can be
    # transformed between livox_frame and any zed_* frame.
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

    lidar_static_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_to_livox',
    arguments=[
        '--x', '0', '--y', '0', '--z', '0.09',
        '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
        '--frame-id', 'base_link', '--child-frame-id', 'livox_frame',
    ],
    )

    return LaunchDescription([
        declare_use_zed,
        zed_wrapper,
        controller,
        zed_mount_tf,
        lidar_static_tf,
        rviz,
    ])
