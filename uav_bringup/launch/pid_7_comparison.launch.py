# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
"""
pid_7_comparison.launch.py
===========================
Igual que pid_7.launch.py pero el logger guarda solo mediciones crudas
(raw_only_mode=True) para comparativa offline de estimadores.

Durante la simulación corre únicamente el estimador H∞ para el control.
Los 2 estimadores (Kalman, H∞) se aplican offline después:

    source install/setup.bash
    python3 src/bearing-estimators/papers/cce2026/reproduce_table.py \\
        /tmp/bearing_comparison/bearing_raw_YYYYMMDD_HHMMSS.mat

El logger graba en:
    bearing_raw_YYYYMMDD_HHMMSS.mat
    par.t, par.g_meas, par.g_gt

USO:
    # Comparativa básica
    ros2 launch uav_bringup pid_7_comparison.launch.py

    # Headless + salida personalizada
    ros2 launch uav_bringup pid_7_comparison.launch.py \\
        headless:=true logger_output_dir:=/tmp/mi_comparativa
"""

import os
import json
import random
import math

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# =============================================================================
# TEMPORIZACIÓN
# =============================================================================
DELAY_SET_POSE  = 2.0
DELAY_BEARING   = 3.5
DELAY_GUI       = 5.0

NOISE_XY  = float(os.environ.get('UAV_IC_NOISE_XY', 0.0))
NOISE_YAW = 0.0

# Gains k0_xy, k1_xy, k0_z, k1_z of the bearing control for the ESP-drone.
ESP_GAINS = (2.2764, 6.7424, 1.923, 3.8366)

ALLOWED_CONNECTIONS = [
    'crazyflie2->crazyflie1',
    'crazyflie2->crazyflie7',
    'crazyflie3->crazyflie4',
    'crazyflie3->crazyflie7',
    'crazyflie6->crazyflie1',
    'crazyflie6->crazyflie7',
    'crazyflie5->crazyflie4',
    'crazyflie5->crazyflie7',
]


def generate_launch_description():

    # ── Args ──────────────────────────────────────────────────────────────────
    drone_arg = DeclareLaunchArgument(
        'drone', default_value='crazyflie',
        description='Drone model: crazyflie | espdrone')
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Gazebo sin GUI (true/false)')
    debug_bearing_arg = DeclareLaunchArgument(
        'debug_bearing', default_value='false',
        description='Debug imagen detector bearing (true/false)')
    auto_sequence_arg = DeclareLaunchArgument(
        'auto_sequence', default_value='false',
        description='Scripted take off and automatic mode, no GUI needed (true/false)')
    estimator_params_arg = DeclareLaunchArgument(
        'estimator_params', default_value='paper',
        description='Estimator parameters: paper (Table I) or tuned')
    run_gui_arg = DeclareLaunchArgument(
        'run_gui', default_value='true',
        description='Lanzar GUI de monitoreo (true/false)')
    logger_output_dir_arg = DeclareLaunchArgument(
        'logger_output_dir', default_value='/tmp/bearing_comparison',
        description='Carpeta de salida para los archivos .mat de comparativa')

    drone             = LaunchConfiguration('drone')
    headless          = LaunchConfiguration('headless')
    debug_bearing     = LaunchConfiguration('debug_bearing')
    run_gui           = LaunchConfiguration('run_gui')
    logger_output_dir = LaunchConfiguration('logger_output_dir')

    # ── Condiciones iniciales ─────────────────────────────────────────────────
    num_drones = 7
    random.seed(int(os.environ.get('UAV_IC_SEED', 42)))
    robot_names = [f"crazyflie{i + 1}" for i in range(num_drones)]
    L = 1.1
    ideal_positions = [
        [  0.3200 * L,   0.3200 * L, 0.0000,  0.0000],
        [  0.6664 * L,   0.1600 * L, 0.0000, math.pi],
        [  0.6664 * L,  -0.1600 * L, 0.0000, math.pi],
        [  0.3200 * L,  -0.3200 * L, 0.0000,  0.0000],
        [ -0.0264 * L,  -0.1600 * L, 0.0000,  0.0000],
        [ -0.0264 * L,   0.1600 * L, 0.0000,  0.0000],
        [  0.3200 * L,   0.0000 * L, 0.0000,  0.0000],
    ]

    initial_conditions = []
    for i in range(num_drones):
        x0, y0, z0, yaw0 = ideal_positions[i]
        if i in [0, 3, 6]:  # the leaders keep their exact positions
            fx, fy, fyaw = x0, y0, yaw0
        else:
            fx   = x0   + random.uniform(-NOISE_XY,  NOISE_XY)
            fy   = y0   + random.uniform(-NOISE_XY,  NOISE_XY)
            fyaw = yaw0 + random.uniform(-NOISE_YAW, NOISE_YAW)
        initial_conditions.append([fx, fy, z0, fyaw])

    initial_conditions_vec = [e for row in initial_conditions for e in row]
    aruco_id_map = [f"{i + 1}={name}" for i, name in enumerate(robot_names)]

    # ── Paquetes ──────────────────────────────────────────────────────────────
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_uav_bringup   = get_package_share_directory('uav_bringup')

    # ── Gazebo ────────────────────────────────────────────────────────────────
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r ', drone, '_vision_7.sdf']
        }.items(),
        condition=UnlessCondition(headless),
    )

    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-s -r --headless-rendering ', drone, '_vision_7.sdf']
        }.items(),
        condition=IfCondition(headless),
    )

    # ── Bridge ────────────────────────────────────────────────────────────────
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution([
                pkg_uav_bringup, 'config', [drone, '_7.yaml']]),
        }],
    )

    # ── Set pose ──────────────────────────────────────────────────────────────
    set_pose_node = Node(
        package='uav_bringup',
        executable='set_pose',
        name='set_pose',
        output='screen',
        parameters=[
            {'use_sim_time': True, 'robot_names':        json.dumps(robot_names)},
            {'initial_conditions': json.dumps(initial_conditions)},
        ],
    )

    wind_node = Node(
        package='uav_bringup',
        executable='wind_node',
        name='wind_node',
        output='screen',
        parameters=[{
            'use_sim_time':       True,
            'base_x':           0.5,
            'base_y':           0.5,
            'base_z':           0.05,
            'variance_x':       0.15,
            'variance_y':       0.06,
            'variance_z':       0.02,
            'gust_probability': 0.05,
        }],
    )

    # The ESP-drone carries a cube of markers above it. The Crazyflie has one marker.
    def for_drone(crazyflie, espdrone):
        return ParameterValue(PythonExpression(
            [str(espdrone), " if '", drone, "' == 'espdrone' else ", str(crazyflie)]),
            value_type=float)

    # ── Controlador ───────────────────────────────────────────────────────────
    controller_node = Node(
        package='uav_controller',
        executable='pid_bearing_only_vision',
        name='uav_controller_node',
        output='screen',
        parameters=[{
            'use_sim_time':       True,
            'robot_names':              robot_names,
            'initial_conditions_vec':   initial_conditions_vec,
            'automatic_duration_sec':   55.0,
            'control_loop_rate':        250.0,
            'takeoff_height':           1.0,
            'takeoff_height_tolerance': 0.05,
            'landing_threshold':        0.05,
            'landing_speed_factor':     0.5,
            'max_linear_velocity':      1.5,
            'max_angular_velocity':     25.5,
            'trajectory_speed':         0.05,
            'trajectory_time_scale':    0.3333,
            'amplitude':                0.5,
            'frequency':                0.02,
            'bearing_loss_timeout_sec': 0.5,
            'pid_bearing.k0_x': for_drone(2.1, ESP_GAINS[0]),  'pid_bearing.k1_x': for_drone(4.0, ESP_GAINS[1]),  'pid_bearing.k2_x': 0.01,
            'pid_bearing.k0_y': for_drone(2.1, ESP_GAINS[0]),  'pid_bearing.k1_y': for_drone(4.0, ESP_GAINS[1]),  'pid_bearing.k2_y': 0.01,
            'pid_bearing.k0_z': for_drone(1.65, ESP_GAINS[2]),  'pid_bearing.k1_z': for_drone(2.2, ESP_GAINS[3]),  'pid_bearing.k2_z': 0.013003,
            'pid.kp_x': 1.5,  'pid.kd_x': 0.05,  'pid.ki_x': 0.2,
            'pid.min_output_x': -1.0,  'pid.max_output_x': 1.0,
            'pid.integrator_min_x': -0.5,  'pid.integrator_max_x': 0.5,
            'pid.kp_y': 1.5,  'pid.kd_y': 0.05,  'pid.ki_y': 0.2,
            'pid.min_output_y': -1.0,  'pid.max_output_y': 1.0,
            'pid.integrator_min_y': -0.5,  'pid.integrator_max_y': 0.5,
            'pid.kp_z': 1.2,  'pid.kd_z': 0.04,  'pid.ki_z': 0.12,
            'pid.min_output_z': -1.0,  'pid.max_output_z': 1.0,
            'pid.integrator_min_z': -0.5,  'pid.integrator_max_z': 0.5,
            'pid.kp_yaw': 0.8,  'pid.kd_yaw': 0.3,  'pid.ki_yaw': 0.05,
            'pid.min_output_yaw': -1.0,  'pid.max_output_yaw': 1.0,
            'pid.integrator_min_yaw': -0.5,  'pid.integrator_max_yaw': 0.5,
        }],
    )

    # ── Bearing node — estimador primario H∞ para control ────────────────────
    get_bearing_node = Node(
        package='uav_perception',
        executable='get_bearing_node',
        name='get_bearing_node_node',
        output='screen',
        parameters=[PathJoinSubstitution([
            get_package_share_directory('uav_bringup'), 'config',
            ['estimators_', LaunchConfiguration('estimator_params'), '.yaml'],
        ]), {
            'use_sim_time':       True,
            'robot_names':                robot_names,
            'world_name':                 'crazyflie',
            'publish_rate_hz':            50.0,
            'image_width':                320,
            'image_height':               240,
            'camera_fov_horizontal_deg':  66.0,
            'aruco_dictionary_id':        9,
            'aruco_marker_length':        for_drone(0.045, 0.05),
            'aruco_cube_side':            for_drone(0.0, 0.0667),
            'aruco_cube_height':          for_drone(0.0, 0.0731),
            'camera_offset_x':            for_drone(0.0, 0.06),
            'camera_pitch_deg':           for_drone(0.0, 6.0),
            'aruco_id_to_robot_name_map': aruco_id_map,
            'kalman_max_prediction_sec':  0.2,
            'thread_count':               -1,
            'allowed_connections':        ALLOWED_CONNECTIONS,
            # Solo estimador H∞ para el controlador (comparativa offline)
            'estimator_type':             'hinf',
            'comparison_mode':            False,
            'debug_mode':   debug_bearing,
        }],
    )

    # ── Logger raw-only — datos crudos para comparativa offline ──────────────
    bearing_logger_node = Node(
        package='uav_tools',
        executable='bearing_logger',
        name='bearing_data_recorder',
        output='screen',
        parameters=[{
            'use_sim_time':       True,
            'robot_names':                robot_names,
            'aruco_id_to_robot_name_map': aruco_id_map,
            'estimator_type':             'raw',
            'raw_only_mode':              True,
            'output_dir':                 logger_output_dir,
            'gz_world_name':              'crazyflie',
            'record_duration_sec':        50.0,
            'auto_start_recording':       False,
        }],
    )

    # ── GUI ───────────────────────────────────────────────────────────────────
    # Scripted take off -> automatic -> land, for runs without the GUI.
    sequencer_node = ExecuteProcess(
        cmd=['ros2', 'run', 'uav_tools', 'server_gui'],
        output='screen',
        name='sequencer',
        condition=IfCondition(LaunchConfiguration('auto_sequence')),
    )

    gui_node = ExecuteProcess(
        cmd=['ros2', 'run', 'uav_tools', 'cce2026_gui_v2', 'hinf'],
        output='screen',
        name='cce2026_gui',
        condition=IfCondition(run_gui),
    )

    # ── Launch description ────────────────────────────────────────────────────
    return LaunchDescription([
        drone_arg,
        headless_arg,
        debug_bearing_arg,
        run_gui_arg,
        estimator_params_arg,
        auto_sequence_arg,
        logger_output_dir_arg,
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        SetEnvironmentVariable('OMP_NUM_THREADS', '20'),
        SetEnvironmentVariable('ODE_NUM_THREADS', '12'),
        SetEnvironmentVariable('GZ_SIM_PHYSICS_THREADS', '16'),

        gz_sim_gui,
        gz_sim_server,
        ros_gz_bridge,

        wind_node,

        TimerAction(period=DELAY_SET_POSE, actions=[set_pose_node]),
        TimerAction(period=DELAY_BEARING,  actions=[get_bearing_node,
                                                    bearing_logger_node]),
        TimerAction(period=DELAY_GUI,      actions=[gui_node]),
        TimerAction(period=DELAY_GUI, actions=[sequencer_node]),

        controller_node,
    ])
