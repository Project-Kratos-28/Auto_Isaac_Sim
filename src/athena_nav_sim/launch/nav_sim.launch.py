"""Kratos autonomy stack on the Isaac Sim digital twin, for driving Nav2 from the Kratos GUI.

localization:=ground_truth (default): no SLAM. Isaac's odom -> base_link plus an identity
map -> odom, and Nav2's global costmap starts blank (no static /map layer): the LiDAR obstacle
layer marks walls and obstacles into it as the rover sees them. Starts Nav2 + rover_bridge only.

localization:=glim starts, all on sim time:
  1. static TF  livox_frame -> imu_link   (T_lidar_imu from the GLIM config)
  2. GLIM + waypoint_manager              /glim_ros/map, /glim_ros/odom, /add_waypoint, ...
     and its TF, depending on isaac_odom_tf:
       true  (default) Isaac's Odom graph keeps publishing odom -> base_link (ground truth).
             GLIM publishes map -> glim_odom -> glim_imu, and glim_map_odom turns that into
             map -> odom, the way AMCL does.
       false Isaac's odom -> base_link TF is turned off; GLIM publishes map -> odom -> base_link
             like on the rover.
  3. pcd2pgm (live, pcd2pgm_sim.yaml)     /glim_ros/map -> /map_glim
     map_seed                             blank /map until pcd2pgm has one, then relays /map_glim
  4. Nav2 (kratos_nav's nodes)            nav2_params_sim.yaml, LiDAR only, sim behavior trees
                                           (Spin recovery allowed, never BackUp)
  5. rover_bridge (sim)                   GUI AUTO/MANUAL gate: cmd_vel_auto -> /cmd_vel

Expects the sim playing and athena_controller bringup.launch.py running (it provides the
base_link -> livox_frame / zed_camera_link mounts, which nav.launch.py republishes with the
same values).

Run:  ros2 launch athena_nav_sim nav_sim.launch.py
      ros2 launch athena_nav_sim nav_sim.launch.py localization:=glim glim_viewer:=false
"""
import importlib.util
import os
import re
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def glim_config_dir(context):
    """Copy of the sim GLIM config with our TF frames and waypoint_manager loaded."""
    src = os.path.expanduser(context.launch_configurations['glim_config'])
    viewer = context.launch_configurations['glim_viewer'].lower() == 'true'
    if context.launch_configurations['isaac_odom_tf'].lower() == 'true':
        # Own frame names so GLIM's tree doesn't fight Isaac's odom -> base_link.
        frames = {'imu_frame_id': 'glim_imu', 'base_frame_id': '', 'odom_frame_id': 'glim_odom'}
    else:
        frames = {'base_frame_id': 'base_link'}
    dst = tempfile.mkdtemp(prefix='athena_glim_config_')
    for name in os.listdir(src):
        with open(os.path.join(src, name)) as f:
            text = f.read()
        if name == 'config_ros.json':
            for key, value in frames.items():
                text = re.sub(rf'"{key}":\s*"[^"]*"', f'"{key}": "{value}"', text)
            if '"libwaypoint_manager.so"' not in text:
                text = text.replace('"librviz_viewer.so"', '"librviz_viewer.so",\n      "libwaypoint_manager.so"', 1)
            if not viewer:
                text = '\n'.join(line for line in text.split('\n') if '"libstandard_viewer.so"' not in line)
        if name == 'config_preprocess.json':
            # The LiDAR sees the rover's own wheel modules (hundreds of points per scan while steering).
            # Nav2's costmaps clear under the footprint, but GLIM keeps them: a trail of phantom
            # obstacles in /glim_ros/map. Crop the rover body (LiDAR frame, LiDAR at the centre).
            text = re.sub(r'"enable_cropbox_filter":\s*\w+', '"enable_cropbox_filter": true', text)
            text = re.sub(r'"crop_bbox_frame":\s*"[^"]*"', '"crop_bbox_frame": "lidar"', text)
            text = re.sub(r'"crop_bbox_min":\s*\[[^\]]*\]', '"crop_bbox_min": [-0.8, -0.75, -0.6]', text)
            text = re.sub(r'"crop_bbox_max":\s*\[[^\]]*\]', '"crop_bbox_max": [0.8, 0.75, 0.5]', text)
        if name == 'config.json' and context.launch_configurations['glim_gpu'].lower() != 'true':
            # GLIM's CUDA modules compete with Isaac's RTX LiDAR for the GPU (LiDAR fell to 0.4 Hz).
            text = re.sub(r'"(config_(?:odometry|sub_mapping|global_mapping))":\s*"\1_gpu\.json"',
                          r'"\1": "\1_cpu.json"', text)
        if name.startswith('config_sub_mapping'):
            # GLIM only publishes /glim_ros/map (-> pcd2pgm -> /map) when a submap closes; at sim
            # speed the rover's 15 keyframes take many metres, so close submaps sooner.
            text = re.sub(r'"max_num_keyframes":\s*\d+',
                          f'"max_num_keyframes": {context.launch_configurations["submap_keyframes"]}', text)
            # OVERLAP adds a keyframe only when a scan overlaps the last ones < max_keyframe_overlap;
            # in a small room the LiDAR sees everything from everywhere, so submaps never close.
            # A keyframe every 0.5 m / 0.5 rad instead.
            text = re.sub(r'"keyframe_update_strategy":\s*"[A-Z_]+"', '"keyframe_update_strategy": "DISPLACEMENT"', text)
            text = re.sub(r'"keyframe_update_interval_trans":\s*[0-9.]+', '"keyframe_update_interval_trans": 0.5', text)
            text = re.sub(r'"keyframe_update_interval_rot":\s*[0-9.]+', '"keyframe_update_interval_rot": 0.5', text)
        with open(os.path.join(dst, name), 'w') as f:
            f.write(text)
    return dst


def lidar_imu_tf_args(config_dir):
    """static_transform_publisher args for livox_frame -> imu_link from T_lidar_imu (TUM x y z qx qy qz qw)."""
    with open(os.path.join(config_dir, 'config_sensors.json')) as f:
        match = re.search(r'"T_lidar_imu":\s*\[([^\]]*)\]', f.read())
    x, y, z, qx, qy, qz, qw = (v.strip() for v in match.group(1).split(','))
    return ['--x', x, '--y', y, '--z', z, '--qx', qx, '--qy', qy, '--qz', qz, '--qw', qw,
            '--frame-id', 'livox_frame', '--child-frame-id', 'imu_link']


def cuda12_runtime_env():
    """cuda12.x GLIM builds need libcudart.so.12 (pip nvidia-cuda-runtime-cu12), see glim/README.md."""
    try:
        import nvidia.cuda_runtime
    except ImportError:
        return {}
    lib = os.path.join(os.path.dirname(nvidia.cuda_runtime.__file__), 'lib')
    return {'LD_LIBRARY_PATH': lib + ':' + os.environ.get('LD_LIBRARY_PATH', '')}


def glim_nodes(context):
    config = glim_config_dir(context)
    imu_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='lidar_to_imu',
        arguments=lidar_imu_tf_args(config))
    # GLIM starts after the static TFs exist, otherwise it can't resolve base_link -> imu_link.
    glim = Node(
        package='glim_ros', executable='glim_rosnode', name='glim_ros', output='screen',
        parameters=[{'config_path': config, 'use_sim_time': True}],
        additional_env=cuda12_runtime_env())
    map_odom = Node(
        package='athena_nav_sim', executable='glim_map_odom', name='glim_map_odom', output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(context.launch_configurations['isaac_odom_tf']))
    return [imu_tf, map_odom, TimerAction(period=3.0, actions=[glim])]


def localization_nodes(context):
    if context.launch_configurations['localization'] == 'ground_truth':
        # Isaac's odometry is perfect, so map and odom are the same frame.
        return [Node(
            package='tf2_ros', executable='static_transform_publisher', name='map_to_odom',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'])]
    pcd2pgm = Node(
        package='pcd2pgm', executable='pcd2pgm_node', name='pcd2pgm', output='screen',
        parameters=[os.path.join(get_package_share_directory('pcd2pgm'), 'config', 'pcd2pgm_sim.yaml'),
                    # GLIM's map keeps the LiDAR's sparse floor returns (floor ~ -0.43 in its map frame,
                    # whose z=0 is the IMU's start height), smeared up to ~0 by the rocker-bogie's
                    # pitching; the costmaps' live raytracing clears them, the map doesn't. So /map
                    # starts higher than the costmaps' obstacle band.
                    {'use_sim_time': True,
                     'thre_z_min': float(context.launch_configurations['map_min_height']),
                     'thre_z_max': float(context.launch_configurations['max_obstacle_height']),
                     'map_topic_name': 'map_glim'}])
    # Blank /map until pcd2pgm's first map, then relays it (Nav2 can't plan without a /map).
    map_seed = Node(
        package='athena_nav_sim', executable='map_seed', name='map_seed', output='screen',
        parameters=[{'use_sim_time': True}])
    return glim_nodes(context) + [pcd2pgm, map_seed]


def kratos_nav_nodes():
    """NAV2_NODES (package, executable, name, remappings) from kratos_nav's nav.launch.py."""
    path = os.path.join(get_package_share_directory('kratos_nav'), 'launch', 'nav.launch.py')
    spec = importlib.util.spec_from_file_location('kratos_nav_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.NAV2_NODES


def nav2_params(context):
    """nav2_params_sim.yaml adjusted for the sim and the chosen localization, as a temporary copy."""
    cfg = context.launch_configurations
    kratos_nav = get_package_share_directory('kratos_nav')
    with open(os.path.join(kratos_nav, 'config', 'nav2_params_sim.yaml')) as f:
        data = yaml.safe_load(f)
    for node in data.values():
        node.setdefault('ros__parameters', {})['use_sim_time'] = True
    bt_dir = os.path.join(get_package_share_directory('athena_nav_sim'), 'behavior_trees')
    data['bt_navigator']['ros__parameters'].update({
        'default_nav_to_pose_bt_xml': os.path.join(bt_dir, 'navigate_spin_no_backup.xml'),
        'default_nav_through_poses_bt_xml': os.path.join(bt_dir, 'navigate_through_poses_spin_no_backup.xml'),
    })
    data['planner_server']['ros__parameters']['GridBased']['lattice_filepath'] = os.path.join(
        kratos_nav, 'config', 'lattice', 'output.json')
    # Spin recovery (the trees above call it). Still no BackUp: the rover never reverses.
    behaviors = data['behavior_server']['ros__parameters']
    behaviors['behavior_plugins'] = ['spin', 'wait']
    behaviors['spin'] = {'plugin': 'nav2_behaviors::Spin'}
    local = data['local_costmap']['local_costmap']['ros__parameters']
    glob = data['global_costmap']['global_costmap']['ros__parameters']
    # LiDAR only: nvblox (ZED depth) isn't installed here.
    local['plugins'] = [name for name in local['plugins'] if name != 'nvblox_layer']
    local.pop('nvblox_layer', None)
    for costmap in (local, glob):
        costmap['obstacle_layer']['lidar']['min_obstacle_height'] = float(cfg['min_obstacle_height'])
        costmap['obstacle_layer']['lidar']['max_obstacle_height'] = float(cfg['max_obstacle_height'])
    # The sim's wheels have no motor deadband; the rover's 0.3 m/s one stalls slow approaches.
    data['velocity_smoother']['ros__parameters']['deadband_velocity'] = [0.0, 0.0, 0.0]
    if cfg['localization'] == 'ground_truth':
        for node in data.values():
            if 'odom_topic' in node.get('ros__parameters', {}):
                node['ros__parameters']['odom_topic'] = '/odom'
        # Blank global map that the LiDAR fills in: no static layer, fixed 50x50 m grid.
        glob['plugins'] = [p for p in glob['plugins'] if p != 'static_layer']
        glob.pop('static_layer', None)
        glob.update({'rolling_window': False, 'width': 50, 'height': 50,
                     'origin_x': -25.0, 'origin_y': -25.0})
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='athena_nav2_', delete=False) as f:
        yaml.safe_dump(data, f)
    return f.name


def nav2_nodes(context):
    """kratos_nav's seven Nav2 nodes with our params. Not via nav.launch.py: it forces its own
    (no-spin) behavior trees onto bt_navigator."""
    params = nav2_params(context)
    cmd_vel_out = 'cmd_vel_auto' if context.launch_configurations['use_bridge'] == 'true' else 'cmd_vel'
    nodes = []
    for package, executable, name, remaps in kratos_nav_nodes():
        # The velocity smoother's output remap is a LaunchConfiguration in nav.launch.py.
        remaps = [(src, dst if isinstance(dst, str) else cmd_vel_out) for src, dst in remaps]
        nodes.append(Node(package=package, executable=executable, name=name, output='screen',
                          parameters=[params], remappings=remaps))
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': [n[2] for n in kratos_nav_nodes()]}]))
    return nodes


def generate_launch_description():
    arg = LaunchConfiguration

    # Mounts, same values as athena_controller bringup.launch.py (which publishes the same links).
    mounts = [
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_link_to_livox',
             arguments=['--z', '0.09', '--frame-id', 'base_link', '--child-frame-id', 'livox_frame']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_link_to_zed',
             arguments=['--x', '0.1', '--z', '0.15', '--frame-id', 'base_link', '--child-frame-id', 'zed_camera_link']),
    ]

    bridge = Node(
        package='athena_nav_sim', executable='rover_bridge', name='rover_bridge', output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(arg('use_bridge')))

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2_nav_sim', output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('athena_nav_sim'), 'rviz', 'nav_sim.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(arg('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false',
                              description='true: RViz with the GLIM map, /map, costmap, plan and footprint'),
        DeclareLaunchArgument('localization', default_value='ground_truth',
                              choices=['ground_truth', 'glim'],
                              description='ground_truth: Isaac odometry, blank global map; glim: SLAM'),
        # The sim LiDAR gets no returns from the ground plane, so every return is an obstacle.
        DeclareLaunchArgument('min_obstacle_height', default_value='-0.5',
                              description='Costmap obstacle band, z in map/odom (m)'),
        DeclareLaunchArgument('max_obstacle_height', default_value='2.5'),
        DeclareLaunchArgument('map_min_height', default_value='0.05',
                              description='glim: lowest z (GLIM map frame) pcd2pgm puts into /map; floor ~-0.43, '
                                          'floor returns smear up to ~0'),
        DeclareLaunchArgument('glim_config', default_value='~/Isaac_sim/glim/glim_config',
                              description='Sim GLIM config dir (copied and patched at launch)'),
        DeclareLaunchArgument('glim_viewer', default_value='true',
                              description="false: GLIM without its OpenGL viewer (saves GPU next to Isaac)"),
        DeclareLaunchArgument('glim_gpu', default_value='false',
                              description="true: GLIM's CUDA modules (they slow Isaac's RTX LiDAR down)"),
        DeclareLaunchArgument('submap_keyframes', default_value='5',
                              description='Keyframes per GLIM submap (rover config: 15); fewer = /map sooner'),
        DeclareLaunchArgument('isaac_odom_tf', default_value='true',
                              description="true: Isaac publishes odom -> base_link, GLIM only map -> odom"),
        DeclareLaunchArgument('use_bridge', default_value='true',
                              description='true: Nav2 only drives in AUTO (GUI button); false: Nav2 -> /cmd_vel directly'),
        OpaqueFunction(function=localization_nodes),
        *mounts,
        OpaqueFunction(function=nav2_nodes),
        bridge,
        rviz,
    ])
