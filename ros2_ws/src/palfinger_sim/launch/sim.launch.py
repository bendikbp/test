from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable, TimerAction, DeclareLaunchArgument, EmitEvent, LogInfo
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import math
import os
import xml.etree.ElementTree as ET


def write_controllers_yaml(path: str, wire_segments: int):
    wire_joint_names = [f'wire_vis_{i:02d}_joint' for i in range(1, wire_segments + 1)]
    lines = [
        'controller_manager:',
        '  ros__parameters:',
        '    update_rate: 100',
        '',
        '    joint_state_broadcaster:',
        '      type: joint_state_broadcaster/JointStateBroadcaster',
        '',
        '    crane_slew_controller:',
        '      type: position_controllers/JointGroupPositionController',
        '',
        '    crane_reach_controller:',
        '      type: position_controllers/JointGroupPositionController',
    ]
    lines.extend([
        '',
        'crane_slew_controller:',
        '  ros__parameters:',
        '    joints:',
        '      - boom_yaw_joint',
        '',
        'crane_reach_controller:',
        '  ros__parameters:',
        '    joints:',
        '      - trolley_joint',
        '      - winch_joint',
        '      - wire_sway_joint',
        '      - winch_drum_joint',
    ])
    lines.extend([f'      - {j}' for j in wire_joint_names])
    with open(path, 'w', encoding='ascii') as f:
        f.write('\n'.join(lines) + '\n')


def gated_on_exit(target_action, success_actions, failure_label: str, shutdown_on_failure: bool = True):
    def _handle_exit(event, _context):
        if event.returncode == 0:
            return success_actions
        if not shutdown_on_failure:
            return [
                LogInfo(msg=f'{failure_label} exited with code {event.returncode}; continuing startup sequence.'),
                *success_actions,
            ]
        return [
            LogInfo(msg=f'{failure_label} failed with exit code {event.returncode}; shutting down launch.'),
            EmitEvent(event=Shutdown(reason=f'{failure_label} failed')),
        ]

    return RegisterEventHandler(OnProcessExit(target_action=target_action, on_exit=_handle_exit))


def load_model_mass_kg(model_sdf_path: str, default_mass_kg: float) -> float:
    try:
        root = ET.parse(model_sdf_path).getroot()
        mass_node = root.find('.//inertial/mass')
        if mass_node is None or mass_node.text is None:
            return default_mass_kg
        return float(mass_node.text.strip())
    except (OSError, ET.ParseError, ValueError):
        return default_mass_kg


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('palfinger_sim')
    desc_pkg = get_package_share_directory('palfinger_description')

    world_profile_map = {
        'calm': os.path.join(gazebo_pkg, 'worlds', 'crane_world_calm.sdf'),
        'moderate': os.path.join(gazebo_pkg, 'worlds', 'crane_world.sdf'),
        'rough': os.path.join(gazebo_pkg, 'worlds', 'crane_world_rough.sdf'),
    }
    default_world_path = world_profile_map['moderate']
    xacro_path = os.path.join(desc_pkg, 'urdf', 'crane.urdf.xacro')
    wire_segments = 32
    wire_curve_exp = 1.5293
    controllers_yaml = os.path.join('/tmp', f'crane_controllers_auto_{wire_segments}.yaml')
    write_controllers_yaml(controllers_yaml, wire_segments)
    hugin_sdf_path = os.path.join(desc_pkg, 'models', 'hugin_platform', 'model.sdf')
    ship_havyard_static_sdf_path = os.path.join(desc_pkg, 'models', 'Ship', 'model_havyard.sdf')
    ship_havyard_dynamic_sdf_path = os.path.join(desc_pkg, 'models', 'Ship', 'model_dynamic_havyard.sdf')
    container_sdf_path = os.path.join(desc_pkg, 'models', 'container', 'model.sdf')
    container_mass_kg = load_model_mass_kg(container_sdf_path, default_mass_kg=1000.0)
    container_dark_blue_sdf_path = os.path.join(desc_pkg, 'models', 'container', 'model_AA.sdf')
    container_dpa_sdf_path = os.path.join(desc_pkg, 'models', 'container', 'model_DPA.sdf')
    container_ss_sdf_path = os.path.join(desc_pkg, 'models', 'container', 'model_SS.sdf')
    robot_description_topic = '/palfinger/robot_description'

    world_sdf = LaunchConfiguration('world_sdf')
    ocean_wave_profile = LaunchConfiguration('ocean_wave_profile')
    crane_x = LaunchConfiguration('crane_x')
    crane_y = LaunchConfiguration('crane_y')
    crane_z = LaunchConfiguration('crane_z')
    crane_yaw = LaunchConfiguration('crane_yaw')
    ship_roll = LaunchConfiguration('ship_roll')
    ship_roll_dynamic = LaunchConfiguration('ship_roll_dynamic')
    ship_pitch = LaunchConfiguration('ship_pitch')
    ship_dynamic = LaunchConfiguration('ship_dynamic')
    havyard_x = LaunchConfiguration('havyard_x')
    havyard_y = LaunchConfiguration('havyard_y')
    havyard_z = LaunchConfiguration('havyard_z')
    havyard_yaw = LaunchConfiguration('havyard_yaw')
    container_blue_x = LaunchConfiguration('container_blue_x')
    container_blue_y = LaunchConfiguration('container_blue_y')
    container_blue_z = LaunchConfiguration('container_blue_z')
    container_blue_yaw = LaunchConfiguration('container_blue_yaw')
    container_x = LaunchConfiguration('container_x')
    container_y = LaunchConfiguration('container_y')
    container_z = LaunchConfiguration('container_z')
    container_yaw = LaunchConfiguration('container_yaw')
    container_dark_blue_x = LaunchConfiguration('container_dark_blue_x')
    container_dark_blue_y = LaunchConfiguration('container_dark_blue_y')
    container_dark_blue_z = LaunchConfiguration('container_dark_blue_z')
    container_dark_blue_roll = LaunchConfiguration('container_dark_blue_roll')
    container_dark_blue_pitch = LaunchConfiguration('container_dark_blue_pitch')
    container_dark_blue_yaw = LaunchConfiguration('container_dark_blue_yaw')
    container_dpa_x = LaunchConfiguration('container_dpa_x')
    container_dpa_y = LaunchConfiguration('container_dpa_y')
    container_dpa_z = LaunchConfiguration('container_dpa_z')
    container_dpa_roll = LaunchConfiguration('container_dpa_roll')
    container_dpa_pitch = LaunchConfiguration('container_dpa_pitch')
    container_dpa_yaw = LaunchConfiguration('container_dpa_yaw')
    container_ss_x = LaunchConfiguration('container_ss_x')
    container_ss_y = LaunchConfiguration('container_ss_y')
    container_ss_z = LaunchConfiguration('container_ss_z')
    container_ss_roll = LaunchConfiguration('container_ss_roll')
    container_ss_pitch = LaunchConfiguration('container_ss_pitch')
    container_ss_yaw = LaunchConfiguration('container_ss_yaw')
    spawn_container_variants = LaunchConfiguration('spawn_container_variants')
    spawn_container_dark_blue = LaunchConfiguration('spawn_container_dark_blue')
    spawn_container_dpa = LaunchConfiguration('spawn_container_dpa')
    spawn_container_ss = LaunchConfiguration('spawn_container_ss')
    command_source = LaunchConfiguration('command_source')
    training_profile = LaunchConfiguration('training_profile')
    enable_snap = LaunchConfiguration('enable_snap')
    enable_camera_viewer = LaunchConfiguration('enable_camera_viewer')
    enable_hmi_view = LaunchConfiguration('enable_hmi_view')
    enable_waves = LaunchConfiguration('enable_waves')
    enable_dp_hold = LaunchConfiguration('enable_dp_hold')
    sea_state_profile = LaunchConfiguration('sea_state_profile')
    wave_amplitude = LaunchConfiguration('wave_amplitude')
    wave_period = LaunchConfiguration('wave_period')
    wave_direction = LaunchConfiguration('wave_direction')

    declare_world_sdf = DeclareLaunchArgument(
        'world_sdf',
        default_value='',
        description='Optional explicit world SDF path. If empty, ocean_wave_profile selects the default world.'
    )
    declare_ocean_wave_profile = DeclareLaunchArgument(
        'ocean_wave_profile',
        default_value='calm',
        description='Visual ocean profile: calm | moderate | rough'
    )
    declare_crane_x = DeclareLaunchArgument('crane_x', default_value='12.0')
    declare_crane_y = DeclareLaunchArgument('crane_y', default_value='8.05')
    declare_crane_z = DeclareLaunchArgument('crane_z', default_value='48.2')
    declare_crane_yaw = DeclareLaunchArgument('crane_yaw', default_value='3.14')
    declare_ship_roll = DeclareLaunchArgument('ship_roll', default_value='1.5708')
    declare_ship_roll_dynamic = DeclareLaunchArgument(
        'ship_roll_dynamic',
        default_value='0.0',
        description='Default dynamic-ship roll. Keep 0.0 for stable buoyancy + DP.'
    )
    declare_ship_pitch = DeclareLaunchArgument('ship_pitch', default_value='0.0')
    declare_ship_dynamic = DeclareLaunchArgument('ship_dynamic', default_value='true')
    declare_havyard_x = DeclareLaunchArgument('havyard_x', default_value='32.0')
    declare_havyard_y = DeclareLaunchArgument('havyard_y', default_value='0.0')
    declare_havyard_z = DeclareLaunchArgument('havyard_z', default_value='-6.0')
    declare_havyard_yaw = DeclareLaunchArgument('havyard_yaw', default_value='3.14')
    declare_container_blue_x = DeclareLaunchArgument('container_blue_x', default_value='35.2')
    declare_container_blue_y = DeclareLaunchArgument('container_blue_y', default_value='11.6')
    declare_container_blue_z = DeclareLaunchArgument('container_blue_z', default_value='4.8')
    declare_container_blue_yaw = DeclareLaunchArgument('container_blue_yaw', default_value='1.5707')
    declare_container_x = DeclareLaunchArgument('container_x', default_value='35.2')
    declare_container_y = DeclareLaunchArgument('container_y', default_value='41.6')
    declare_container_z = DeclareLaunchArgument('container_z', default_value='16.00')
    declare_container_yaw = DeclareLaunchArgument('container_yaw', default_value='1.5707')
    declare_container_dark_blue_x = DeclareLaunchArgument('container_dark_blue_x', default_value='36.0')
    declare_container_dark_blue_y = DeclareLaunchArgument('container_dark_blue_y', default_value='17.6')
    declare_container_dark_blue_z = DeclareLaunchArgument('container_dark_blue_z', default_value='3.5')
    declare_container_dark_blue_roll = DeclareLaunchArgument('container_dark_blue_roll', default_value='1.5707')
    declare_container_dark_blue_pitch = DeclareLaunchArgument('container_dark_blue_pitch', default_value='0.0')
    declare_container_dark_blue_yaw = DeclareLaunchArgument('container_dark_blue_yaw', default_value='3.14')
    declare_container_dpa_x = DeclareLaunchArgument('container_dpa_x', default_value='31.2')
    declare_container_dpa_y = DeclareLaunchArgument('container_dpa_y', default_value='17.6')
    declare_container_dpa_z = DeclareLaunchArgument('container_dpa_z', default_value='3.5')
    declare_container_dpa_roll = DeclareLaunchArgument('container_dpa_roll', default_value='1.5707')
    declare_container_dpa_pitch = DeclareLaunchArgument('container_dpa_pitch', default_value='0.0')
    declare_container_dpa_yaw = DeclareLaunchArgument('container_dpa_yaw', default_value='3.14')
    declare_container_ss_x = DeclareLaunchArgument('container_ss_x', default_value='28.4')
    declare_container_ss_y = DeclareLaunchArgument('container_ss_y', default_value='3.6')
    declare_container_ss_z = DeclareLaunchArgument('container_ss_z', default_value='3.5')
    declare_container_ss_roll = DeclareLaunchArgument('container_ss_roll', default_value='1.57')
    declare_container_ss_pitch = DeclareLaunchArgument('container_ss_pitch', default_value='0.0')
    declare_container_ss_yaw = DeclareLaunchArgument('container_ss_yaw', default_value='1.5707')
    declare_spawn_container_variants = DeclareLaunchArgument(
        'spawn_container_variants',
        default_value='true',
        description='Spawn AA/DPA/SS container variants for visual comparison.'
    )
    declare_spawn_container_dark_blue = DeclareLaunchArgument(
        'spawn_container_dark_blue',
        default_value='false',
        description='Spawn dark blue container variant.'
    )
    declare_spawn_container_dpa = DeclareLaunchArgument(
        'spawn_container_dpa',
        default_value='false',
        description='Spawn DPA container variant.'
    )
    declare_spawn_container_ss = DeclareLaunchArgument(
        'spawn_container_ss',
        default_value='false',
        description='Spawn SS container variant.'
    )
    declare_command_source = DeclareLaunchArgument(
        'command_source',
        default_value='joystick',
        description='Control source: joystick, terminal_rate, terminal_target, or terminal_operator_training.'
    )
    declare_training_profile = DeclareLaunchArgument(
        'training_profile',
        default_value='off',
        description=(
            'Training disturbance profile: off, gusty_wind, sway_training, recovery, '
            'pendulum_kick, sudden_yaw_reversal, hoist_snag_release, or custom. '
            'Terminal shortcuts: 1..5, 0=off.'
        )
    )
    declare_enable_snap = DeclareLaunchArgument(
        'enable_snap',
        default_value='true',
        description='Enable snap manager/executor/targets helper nodes.'
    )
    declare_enable_camera_viewer = DeclareLaunchArgument(
        'enable_camera_viewer',
        default_value='true',
        description='Enable the interactive camera viewer window.'
    )
    declare_enable_hmi_view = DeclareLaunchArgument(
        'enable_hmi_view',
        default_value='true',
        description='Enable the dedicated HMI range monitor window.'
    )
    declare_enable_waves = DeclareLaunchArgument(
        'enable_waves',
        default_value='true',
        description='Enable synthetic wave motion profile.'
    )
    declare_enable_dp_hold = DeclareLaunchArgument(
        'enable_dp_hold',
        default_value='true',
        description='Enable DP hold in X/Y/Yaw.'
    )
    declare_sea_state_profile = DeclareLaunchArgument(
        'sea_state_profile',
        default_value='calm',
        description='Wave profile: calm | moderate | rough'
    )
    declare_wave_amplitude = DeclareLaunchArgument(
        'wave_amplitude',
        default_value='-1.0',
        description='Override wave amplitude in meters. Set < 0 to use profile.'
    )
    declare_wave_period = DeclareLaunchArgument(
        'wave_period',
        default_value='-1.0',
        description='Override wave period in seconds. Set < 0 to use profile.'
    )
    declare_wave_direction = DeclareLaunchArgument(
        'wave_direction',
        default_value='90.0',
        description='Wave direction in degrees relative to ship forward axis. Havyard currently uses 90 for bow-up/down visual motion.'
    )
    winch_min = 0.4
    winch_max = 105.0
    wire_multipliers = [
        math.pow(float(i) / float(wire_segments), wire_curve_exp)
        for i in range(1, wire_segments + 1)
    ]
    wire_joint_names = [f'wire_vis_{i:02d}_joint' for i in range(1, wire_segments + 1)]
    reach_joint_names = [
        'trolley_joint',
        'winch_joint',
        'wire_sway_joint',
        'winch_drum_joint',
    ] + wire_joint_names
    expected_reach_joint_count = 4 + wire_segments
    if len(reach_joint_names) != expected_reach_joint_count:
        raise RuntimeError(
            f"Reach joint configuration is inconsistent: expected {expected_reach_joint_count} "
            f"joints for wire_segments={wire_segments}, got {len(reach_joint_names)}"
        )
    wire_mimic_rules = [
        f'{joint_name}:winch_joint:{multiplier:.10f}:0.0'
        for joint_name, multiplier in zip(wire_joint_names, wire_multipliers)
    ]
    wire_lower_limits = [winch_min * multiplier for multiplier in wire_multipliers]
    wire_upper_limits = [winch_max * multiplier for multiplier in wire_multipliers]

    # 1) Gazebo resource paths (models, meshes, etc.)
    # Keep both the package dir and its parent share dir:
    # - package://crane_description/... resolves via desc_pkg
    # - model://crane_description/... resolves via dirname(desc_pkg)
    desc_pkg_parent = os.path.dirname(desc_pkg)
    desc_models_dir = os.path.join(desc_pkg, 'models')
    existing_resource_path = EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')
    gz_resource_path = [desc_pkg_parent, ':', desc_pkg, ':', desc_models_dir, ':', existing_resource_path]

    # 2) Gazebo system plugin path (where libgz_* system plugins live)
    # This helps Gazebo find libgz_ros2_control-system.so reliably (esp. in VMs).
    existing_plugin_path = EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', default_value='')
    gz_plugin_path = ['/opt/ros/jazzy/lib', ':', existing_plugin_path]

    havyard_sdf_path = PythonExpression([
        "'",
        ship_dynamic,
        "' == 'true' and '",
        ship_havyard_dynamic_sdf_path,
        "' or '",
        ship_havyard_static_sdf_path,
        "'",
    ])
    ship_roll_selected = PythonExpression([
        "'",
        ship_dynamic,
        "' == 'true' and '",
        ship_roll_dynamic,
        "' or '",
        ship_roll,
        "'",
    ])

    create_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'crane',
            '-topic', robot_description_topic,
            '-x', crane_x, '-y', crane_y, '-z', crane_z,
            '-R', '0', '-P', '0', '-Y', crane_yaw
        ]
    )
    spawn_hugin_platform = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'hugin_platform',
            '-file', hugin_sdf_path,
            '-x', '0', '-y', '0', '-z', '0.0',
        ]
    )
    spawn_havyard = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'havyard_ship',
            '-file', havyard_sdf_path,
            '-x', havyard_x, '-y', havyard_y, '-z', havyard_z,
            '-R', ship_roll_selected, '-P', ship_pitch,
            '-Y', havyard_yaw,
        ]
    )
    spawn_container = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'container_blue',
            '-file', container_sdf_path,
            '-x', container_blue_x, '-y', container_blue_y, '-z', container_blue_z,
            '-Y', container_blue_yaw,
        ]
    )
    # spawn_container_dark_blue_node = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     output='screen',
    #     condition=IfCondition(
    #         PythonExpression([
    #             "'",
    #             spawn_container_variants,
    #             "' == 'true' or '",
    #             spawn_container_dark_blue,
    #             "' == 'true'",
    #         ])
    #     ),
    #     arguments=[
    #         '-name', 'container_dblue',
    #         '-file', container_dark_blue_sdf_path,
    #         '-x', container_dark_blue_x, '-y', container_dark_blue_y, '-z', container_dark_blue_z,
    #         '-R', container_dark_blue_roll, '-P', container_dark_blue_pitch, '-Y', container_dark_blue_yaw,
    #     ],
    # )
    # spawn_container_dpa_node = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     output='screen',
    #     condition=IfCondition(
    #         PythonExpression([
    #             "'",
    #             spawn_container_variants,
    #             "' == 'true' or '",
    #             spawn_container_dpa,
    #             "' == 'true'",
    #         ])
    #     ),
    #     arguments=[
    #         '-name', 'container_red',
    #         '-file', container_dpa_sdf_path,
    #         '-x', container_dpa_x, '-y', container_dpa_y, '-z', container_dpa_z,
    #         '-R', container_dpa_roll, '-P', container_dpa_pitch, '-Y', container_dpa_yaw,
    #     ],
    # )
    # spawn_container_ss_node = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     output='screen',
    #     condition=IfCondition(
    #         PythonExpression([
    #             "'",
    #             spawn_container_variants,
    #             "' == 'true' or '",
    #             spawn_container_ss,
    #             "' == 'true'",
    #         ])
    #     ),
    #     arguments=[
    #         '-name', 'container_yellow',
    #         '-file', container_ss_sdf_path,
    #         '-x', container_ss_x, '-y', container_ss_y, '-z', container_ss_z,
    #         '-R', container_ss_roll, '-P', container_ss_pitch, '-Y', container_ss_yaw,
    #     ],
    # )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120.0',
            '--switch-timeout', '120.0',
            '--service-call-timeout', '120.0',
        ]
    )

    crane_slew_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'crane_slew_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120.0',
            '--switch-timeout', '120.0',
            '--service-call-timeout', '120.0',
        ]
    )

    crane_reach_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'crane_reach_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120.0',
            '--switch-timeout', '120.0',
            '--service-call-timeout', '120.0',
        ]
    )

    teleop_joy_node = Node(
        package='palfinger_teleop',
        executable='teleop_joy',
        output='screen',
        condition=IfCondition(PythonExpression(["'", command_source, "' == 'joystick'"])),
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {'use_sim_time': True},
            {
                'joy_topic': '/joy',
                'cmd_topic': '/crane/cmd',
                'axis_slew': 0,
                # Right stick horizontal: trolley (left/right).
                'axis_boom': 3,
                # Right stick vertical: winch (up/down).
                'axis_winch': 4,
                'deadman_button': 3,
                'deadman_buttons': [3],
                'deadman_toggle': True,
                'require_deadman': True,
                'deadzone': 0.08,
                'deadzone_slew': 0.08,
                # Trolley axis needs a stronger deadzone to reject stick drift.
                'deadzone_boom': 0.12,
                'deadzone_winch': 0.08,
                'expo_slew': 1.60,
                'expo_boom': 1.00,
                'expo_winch': 1.00,
                'scale_slew': 1.00,
                'scale_boom': 0.85,
                # Invert so stick up reels in (shorter wire).
                'scale_winch': -0.75,
                'publish_hz': 50.0,
            }
        ]
    )

    terminal_rate_node = Node(
        package='palfinger_teleop',
        executable='teleop_operator_training',
        output='screen',
        condition=IfCondition(PythonExpression(["'", command_source, "' == 'terminal_rate'"])),
        parameters=[
            {'use_sim_time': True},
            {
                'mode': 'rate',
                'cmd_topic': '/crane/priority_cmd',
                'target_topic': '/crane/priority_target',
                'disturbance_topic': '/crane/priority_disturbance_cmd',
                'publish_hz': 50.0,
            }
        ]
    )

    terminal_target_node = Node(
        package='palfinger_teleop',
        executable='teleop_operator_training',
        output='screen',
        condition=IfCondition(PythonExpression(["'", command_source, "' == 'terminal_target'"])),
        parameters=[
            {'use_sim_time': True},
            {
                'mode': 'target',
                'cmd_topic': '/crane/priority_cmd',
                'target_topic': '/crane/priority_target',
                'disturbance_topic': '/crane/priority_disturbance_cmd',
                'publish_hz': 50.0,
            }
        ]
    )

    terminal_operator_training_node = Node(
        package='palfinger_teleop',
        executable='teleop_operator_training',
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'",
                command_source,
                "' == 'terminal_operator_training' or '",
                training_profile,
                "' != 'off'",
            ])
        ),
        parameters=[
            {'use_sim_time': True},
            {
                'mode': 'operator_training',
                'cmd_topic': '/crane/priority_cmd',
                'target_topic': '/crane/priority_target',
                'disturbance_topic': '/crane/priority_disturbance_cmd',
                'external_command_topic': '/crane/training_command',
                'training_profile': training_profile,
                'publish_hz': 50.0,
            }
        ]
    )

    crane_slew_bridge_node = Node(
        package='palfinger_control',
        executable='crane_controller',
        name='crane_slew_cmd_bridge',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {
                'cmd_in_topic': '/crane/cmd',
                'priority_cmd_in_topic': '/crane/priority_cmd',
                'disturbance_cmd_topic': '/crane/disturbance_cmd',
                'priority_disturbance_cmd_topic': '/crane/priority_disturbance_cmd',
                'target_in_topic': '/crane/target',
                'priority_target_in_topic': '/crane/priority_target',
                'joint_states_topic': '/joint_states',
                'cmd_out_topic': '/crane_slew_controller/commands',
                'joint_names': [
                    'boom_yaw_joint',
                ],
                'idx_slew': 0,
                'idx_boom': -1,
                'idx_winch': -1,
                'idx_sway': -1,
                'max_slew_vel': 0.10,
                'max_slew_accel': 0.35,
                'max_boom_vel': 1.00,
                'max_winch_vel': 0.8167,
                'max_winch_accel': 3.00,
                'reach_aware_slew': False,
                'min_slew_scale_at_max_reach': 0.75,
                'reach_weight_trolley': 0.8,
                'reach_weight_winch': 0.2,
                'sway_joint_name': 'wire_sway_joint',
                'sway_brake_gain': 20.0,
                'sway_soft_limit': 0.003,
                'sway_hard_limit': 0.0060,
                'slew_zero_threshold': 0.05,
                'boom_zero_threshold': 0.20,
                'winch_zero_threshold': 0.04,
                'publish_hz': 50.0,
                'timeout_sec': 0.25,
                'wait_for_joint_states': True,
                'startup_target_slew': 0.0,
                'startup_target_boom': 0.0,
                'startup_target_winch': winch_min,
                'lower_limits': [
                    -3.14159,
                ],
                'max_target_error': [
                    0.10,
                ],
                'upper_limits': [
                    3.14159,
                ],
            }
        ]
    )

    crane_reach_bridge_node = Node(
        package='palfinger_control',
        executable='crane_controller',
        name='crane_reach_cmd_bridge',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {
                'cmd_in_topic': '/crane/cmd',
                'priority_cmd_in_topic': '/crane/priority_cmd',
                'disturbance_cmd_topic': '/crane/disturbance_cmd',
                'priority_disturbance_cmd_topic': '/crane/priority_disturbance_cmd',
                'target_in_topic': '/crane/target',
                'priority_target_in_topic': '/crane/priority_target',
                'joint_states_topic': '/joint_states',
                'cmd_out_topic': '/crane_reach_controller/commands',
                'joint_names': reach_joint_names,
                'idx_slew': -1,
                'idx_boom': 0,
                'idx_winch': 1,
                'idx_sway': 2,
                'max_slew_vel': 0.10,
                'max_slew_accel': 0.35,
                'max_boom_vel': 1.00,
                'max_winch_vel': 0.8167,
                'max_winch_accel': 3.00,
                'reach_aware_slew': False,
                'min_slew_scale_at_max_reach': 0.75,
                'reach_weight_trolley': 0.8,
                'reach_weight_winch': 0.2,
                'sway_joint_name': 'wire_sway_joint',
                'sway_brake_gain': 20.0,
                'sway_soft_limit': 0.003,
                'sway_hard_limit': 0.0060,
                'slew_zero_threshold': 0.05,
                'boom_zero_threshold': 0.20,
                'winch_zero_threshold': 0.04,
                'mimic_rules': [
                    'wire_sway_joint:winch_joint:0.0:0.0',
                    'winch_drum_joint:winch_joint:8.0:0.0',
                ] + wire_mimic_rules,
                'publish_hz': 50.0,
                'timeout_sec': 0.25,
                'wait_for_joint_states': True,
                'startup_target_slew': 0.0,
                'startup_target_boom': 0.0,
                'startup_target_winch': winch_min,
                'lower_limits': [
                    -28.0,
                    winch_min,
                    -0.0010,
                    -1000.0,
                ] + wire_lower_limits,
                'max_target_error': [
                    0.08,
                    0.75,
                    -1.0,
                    -1.0,
                ] + ([-1.0] * len(wire_joint_names)),
                'upper_limits': [
                    2.0,
                    winch_max,
                    0.0010,
                    1000.0,
                ] + wire_upper_limits,
            }
        ]
    )

    ship_dp_hold_node = Node(
        package='palfinger_control',
        executable='ship_dp_hold',
        name='ship_dp_hold',
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'",
                enable_waves,
                "' == 'true' or '",
                enable_dp_hold,
                "' == 'true'",
            ])
        ),
        parameters=[
            {'use_sim_time': True},
            {
                'ship_model_name': 'havyard_ship',
                'enable_waves': enable_waves,
                'enable_dp_hold': enable_dp_hold,
                'sea_state_profile': sea_state_profile,
                'wave_amplitude': wave_amplitude,
                'wave_period': wave_period,
                'wave_direction_deg': wave_direction,
                'swap_roll_pitch_axes': True,
                'heave_scale': 0.30,
                'max_heave_vel': 0.35,
                'kp_attitude': 0.45,
                'kd_attitude': 0.90,
                'max_attitude_correction': 0.18,
                'kp_xy': 0.9,
                'kd_xy': 1.8,
                'kp_yaw': 1.6,
                'kd_yaw': 2.2,
                'max_linear_correction': 1.5,
                'max_angular_correction': 0.35,
                'home_x': havyard_x,
                'home_y': havyard_y,
                'home_z': havyard_z,
                'home_roll': ship_roll_selected,
                'home_pitch': ship_pitch,
                'home_yaw': havyard_yaw,
            }
        ],
    )

    snap_manager_node = Node(
        package='palfinger_teleop',
        executable='snap_manager',
        output='screen',
        condition=IfCondition(enable_snap),
        parameters=[
            {'use_sim_time': True},
            {
                'joy_topic': '/joy',
                'targets_topic': '/snap/targets',
                'command_topic': '/snap/command',
                'state_topic': '/snap/state',
                'hook_frame': 'hook_link',
                'world_frame': 'world',
                'snap_button': 2,
                'default_max_snap_distance': 1.5,
                'crane_world_x': crane_x,
                'crane_world_y': crane_y,
                'crane_world_z': crane_z,
                'crane_world_yaw': crane_yaw,
                'allowed_tags': ['container', 'ship', 'payload', 'training'],
                'attachable_tags': ['container'],
            }
        ]
    )

    snap_executor_node = Node(
        package='palfinger_teleop',
        executable='snap_executor',
        output='screen',
        condition=IfCondition(enable_snap),
        parameters=[
            {'use_sim_time': True},
            {
                'command_topic': '/snap/command',
                'hook_frame': 'hook_link',
                'world_frame': 'world',
                'set_pose_service': '/world/crane_world/set_pose',
                'update_hz': 80.0,
                'container_yaw': container_blue_yaw,
                'crane_world_x': crane_x,
                'crane_world_y': crane_y,
                'crane_world_z': crane_z,
                'crane_world_yaw': crane_yaw,
                'hook_clearance_z': 0.50,
                'follow_alpha': 0.45,
                'container_model_sdf': container_sdf_path,
                'create_service': '/world/crane_world/create',
                'remove_service': '/world/crane_world/remove',
                'container_pose_topic': '/snap/container_pose',
                'replace_model_on_attach': True,
                'detach_spawn_lift_z': 0.10,
            }
        ]
    )

    snap_target_provider_node = Node(
        package='palfinger_teleop',
        executable='snap_target_provider',
        output='screen',
        condition=IfCondition(enable_snap),
        parameters=[
            {'use_sim_time': True},
            {
                'targets_topic': '/snap/targets',
                'publish_hz': 5.0,
                'ship_name': 'havyard_ship',
                'ship_tag': 'ship',
                'ship_x': havyard_x,
                'ship_y': havyard_y,
                'ship_z': havyard_z,
                'ship_roll': ship_roll_selected,
                'ship_pitch': ship_pitch,
                'ship_yaw': havyard_yaw,
                'ship_max_snap_distance': 2.0,
                'container_name': 'container_blue',
                'container_tag': 'container',
                'container_x': container_blue_x,
                'container_y': container_blue_y,
                'container_z': container_blue_z,
                'container_yaw': container_blue_yaw,
                'container_max_snap_distance': 2.25,
                'container_pose_topic': '/snap/container_pose',
            }
        ]
    )

    camera_viewer_node = Node(
        package='palfinger_teleop',
        executable='camera_viewer',
        output='screen',
        condition=IfCondition(enable_camera_viewer),
        parameters=[
            {'use_sim_time': True},
            {
                'front_topic': '/crane/boom_tip_camera/image',
                'left_topic': '/crane/hook_mid_camera/image',
                'right_topic': '/crane/cabin_camera/image',
                'window_name': 'Crane Cameras',
                'joy_topic': '/joy',
                'joy_enabled': True,
                'joy_cycle_button': 0,
                'joy_zoom_modifier_button': 5,
                'joy_pan_axis': 6,
                'joy_tilt_zoom_axis': 7,
                'joy_pan_axis_sign': -1.0,
                'joy_axis_threshold': 0.5,
                'joy_repeat_hz': 8.0,
                'front_pan_cmd_topic': '/crane/boom_tip_camera/pan_cmd',
                'front_tilt_cmd_topic': '/crane/boom_tip_camera/tilt_cmd',
                'left_pan_cmd_topic': '/crane/hook_mid_camera/pan_cmd',
                'left_tilt_cmd_topic': '/crane/hook_mid_camera/tilt_cmd',
                'right_pan_cmd_topic': '/crane/cabin_camera/pan_cmd',
                'right_tilt_cmd_topic': '/crane/cabin_camera/tilt_cmd',
            }
        ]
    )

    hmi_range_monitor_node = Node(
        package='palfinger_teleop',
        executable='hmi_range_monitor',
        output='screen',
        condition=IfCondition(enable_hmi_view),
        parameters=[
            {'use_sim_time': True},
            {
                'ranges_topic': '/crane/hmi/ranges',
                'hook_frame': 'hook_link',
                'world_frame': 'world',
                'publish_hz': 10.0,
                'crane_world_x': crane_x,
                'crane_world_y': crane_y,
                'crane_world_z': crane_z,
                'crane_world_yaw': crane_yaw,
                'container_pose_topic': '/snap/container_pose',
                'ship_odom_topic': '/model/havyard_ship/odometry',
                'ship_x': havyard_x,
                'ship_y': havyard_y,
                'ship_z': havyard_z,
                'ship_roll': ship_roll_selected,
                'ship_pitch': ship_pitch,
                'ship_yaw': havyard_yaw,
                'platform_x': 0.0,
                'platform_y': 0.0,
                'platform_z': 0.0,
                'platform_roll': 0.0,
                'platform_pitch': 0.0,
                'platform_yaw': 0.0,
                'attached_container_mass_kg': container_mass_kg,
            }
        ]
    )

    hmi_range_viewer_node = Node(
        package='palfinger_teleop',
        executable='hmi_range_viewer',
        output='screen',
        condition=IfCondition(enable_hmi_view),
        parameters=[
            {'use_sim_time': True},
            {
                'ranges_topic': '/crane/hmi/ranges',
                'window_name': 'Palfinger HMI Ranges',
                'refresh_hz': 15.0,
            }
        ]
    )

    crane_ptz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/crane/boom_tip_camera/pan_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/crane/boom_tip_camera/tilt_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/crane/hook_mid_camera/pan_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/crane/hook_mid_camera/tilt_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/crane/cabin_camera/pan_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/crane/cabin_camera/tilt_cmd@std_msgs/msg/Float64]gz.msgs.Double',
        ],
    )

    crane_camera_image_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/crane/boom_tip_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/crane/hook_mid_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/crane/cabin_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
    )

    set_pose_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/world/crane_world/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ],
    )

    create_entity_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/world/crane_world/create@ros_gz_interfaces/srv/SpawnEntity',
        ],
    )

    remove_entity_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/world/crane_world/remove@ros_gz_interfaces/srv/DeleteEntity',
        ],
    )

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', PythonExpression([
            "'",
            world_sdf,
            "' != '' and '",
            world_sdf,
            "' or '",
            world_profile_map['calm'],
            "' if '",
            ocean_wave_profile,
            "' == 'calm' else '",
            world_profile_map['rough'],
            "' if '",
            ocean_wave_profile,
            "' == 'rough' else '",
            default_world_path,
            "'"
        ])],
        output='screen'
    )

    spawn_entity_after_delay = TimerAction(
        period=10.0,
        actions=[create_entity]
    )
    spawn_scene_after_delay = TimerAction(
        period=6.0,
        actions=[
            spawn_hugin_platform,
            spawn_havyard,
            spawn_container,
            # spawn_container_dark_blue_node,
            # spawn_container_dpa_node,
            # spawn_container_ss_node,
        ]
    )

    start_jsb_after_entity = gated_on_exit(
        create_entity,
        [
            TimerAction(
                period=5.0,
                actions=[joint_state_spawner],
            )
        ],
        'crane entity spawn',
    )

    start_slew_after_jsb = gated_on_exit(
        joint_state_spawner,
        [
            TimerAction(
                period=3.0,
                actions=[crane_slew_spawner],
            )
        ],
        'joint_state_broadcaster spawner',
        shutdown_on_failure=False,
    )

    start_reach_after_slew = gated_on_exit(
        crane_slew_spawner,
        [
            TimerAction(
                period=3.0,
                actions=[crane_reach_spawner],
            )
        ],
        'crane_slew_controller spawner',
        shutdown_on_failure=False,
    )

    start_control_after_reach = gated_on_exit(
        crane_reach_spawner,
        [
            crane_slew_bridge_node,
            crane_reach_bridge_node,
            teleop_joy_node,
            terminal_rate_node,
            terminal_target_node,
            terminal_operator_training_node,
            snap_target_provider_node,
            snap_manager_node,
            snap_executor_node,
            camera_viewer_node,
            hmi_range_monitor_node,
            hmi_range_viewer_node,
            ship_dp_hold_node,
        ],
        'crane_reach_controller spawner',
        shutdown_on_failure=False,
    )

    return LaunchDescription([
        # Force deterministic GUI rendering path to avoid intermittent gray-background fallback.
        SetEnvironmentVariable(name='GZ_RENDER_ENGINE', value='ogre2'),
        SetEnvironmentVariable(name='__NV_PRIME_RENDER_OFFLOAD', value='1'),
        SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia'),
        declare_world_sdf,
        declare_ocean_wave_profile,
        declare_crane_x,
        declare_crane_y,
        declare_crane_z,
        declare_crane_yaw,
        declare_ship_roll,
        declare_ship_roll_dynamic,
        declare_ship_pitch,
        declare_ship_dynamic,
        declare_havyard_x,
        declare_havyard_y,
        declare_havyard_z,
        declare_havyard_yaw,
        declare_container_blue_x,
        declare_container_blue_y,
        declare_container_blue_z,
        declare_container_blue_yaw,
        declare_container_x,
        declare_container_y,
        declare_container_z,
        declare_container_yaw,
        declare_container_dark_blue_x,
        declare_container_dark_blue_y,
        declare_container_dark_blue_z,
        declare_container_dark_blue_roll,
        declare_container_dark_blue_pitch,
        declare_container_dark_blue_yaw,
        declare_container_dpa_x,
        declare_container_dpa_y,
        declare_container_dpa_z,
        declare_container_dpa_roll,
        declare_container_dpa_pitch,
        declare_container_dpa_yaw,
        declare_container_ss_x,
        declare_container_ss_y,
        declare_container_ss_z,
        declare_container_ss_roll,
        declare_container_ss_pitch,
        declare_container_ss_yaw,
        declare_spawn_container_variants,
        declare_spawn_container_dark_blue,
        declare_spawn_container_dpa,
        declare_spawn_container_ss,
        declare_command_source,
        declare_training_profile,
        declare_enable_snap,
        declare_enable_camera_viewer,
        declare_enable_hmi_view,
        declare_enable_waves,
        declare_enable_dp_hold,
        declare_sea_state_profile,
        declare_wave_amplitude,
        declare_wave_period,
        declare_wave_direction,
        SetEnvironmentVariable(name='ROS_AUTOMATIC_DISCOVERY_RANGE', value='LOCALHOST'),
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_resource_path),
        SetEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH', value=gz_plugin_path),

        # Start Gazebo
        gz_sim,

        # Publish robot_description from xacro
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='palfinger_robot_state_publisher',
            namespace='palfinger',
            output='screen',
            remappings=[
                ('/joint_states', '/joint_states'),
                ('joint_states', '/joint_states'),
                ('/joint_states_sanitized', '/joint_states'),
                ('joint_states_sanitized', '/joint_states'),
            ],
            parameters=[
                {'use_sim_time': True},
                {
                    'robot_description': ParameterValue(
                        Command([
                            FindExecutable(name='xacro'), ' ', xacro_path,
                            ' ', 'controllers_yaml:=', controllers_yaml,
                            ' ', 'wire_segments:=', str(wire_segments),
                            ' ', 'wire_curve_exp:=', str(wire_curve_exp),
                        ]),
                        value_type=str
                    ),
                }
            ]
        ),

        # Bridge Gazebo simulation clock to ROS 2 /clock.
        # Without this, nodes using use_sim_time can warn and timeout behavior is less reliable.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/model/havyard_ship/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/model/havyard_ship/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            ],
        ),

        crane_ptz_bridge_node,
        crane_camera_image_bridge_node,
        set_pose_bridge_node,
        create_entity_bridge_node,
        remove_entity_bridge_node,

        # Joystick input driver
        Node(
            package='joy',
            executable='joy_node',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {
                    # Keep publishing the latest joystick state to avoid stale command behavior.
                    'autorepeat_rate': 50.0,
                    'deadzone': 0.05,
                }
            ]
        ),

        # Startup sequence:
        # create entity -> joint_state_broadcaster -> crane controllers -> control/teleop
        spawn_scene_after_delay,
        spawn_entity_after_delay,
        start_jsb_after_entity,
        start_slew_after_jsb,
        start_reach_after_slew,
        start_control_after_reach,
    ])
