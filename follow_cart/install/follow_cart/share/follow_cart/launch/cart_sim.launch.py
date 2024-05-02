

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'follow_cart'
    convoy_name_in_model = 'convoy'
    fc1_name_in_model = 'follow_cart_1'
    fc2_name_in_model = 'follow_cart_2'
    fc3_name_in_model = 'follow_cart_3'

    # 로봇 초기 생성 위치
    convoy_spawn_x_val = '0.0'
    convoy_spawn_y_val = '0.0'
    convoy_spawn_z_val = '1.0'
    convoy_spawn_yaw_val = '0.0'

    fc1_spawn_x_val = '-1.0'
    fc1_spawn_y_val = '0.0'
    fc1_spawn_z_val = '0.5'
    fc1_spawn_yaw_val = '0.0'

    fc2_spawn_x_val = '-1.0'
    fc2_spawn_y_val = '0.5'
    fc2_spawn_z_val = '0.5'
    fc2_spawn_yaw_val = '0.0'

    fc3_spawn_x_val = '-1.0'
    fc3_spawn_y_val = '-0.5'
    fc3_spawn_z_val = '0.5'
    fc3_spawn_yaw_val = '0.0'

    # urdf 파일 경로
    pkg_share = get_package_share_directory(package_name)
    convoy_urdf_path = os.path.join(pkg_share, 'urdf', 'convoy', 'jetbot.urdf')
    fc_urdf_path = os.path.join(pkg_share, 'urdf', 'follow_cart', 'turtlebot3_waffle_pi.urdf')

    # ekf 파일 경로
    localization_yaml_convoy = os.path.join(pkg_share, 'config', 'convoy_ekf.yaml')
    localization_yaml_fc1 = os.path.join(pkg_share, 'config', 'fc1_ekf.yaml')
    localization_yaml_fc2 = os.path.join(pkg_share, 'config', 'fc2_ekf.yaml')
    localization_yaml_fc3 = os.path.join(pkg_share, 'config', 'fc3_ekf.yaml')

    # convoy nav2 파일 경로
    amcl_yaml_convoy = os.path.join(pkg_share, 'config', 'convoy_amcl_config.yaml')
    controller_yaml_convoy = os.path.join(pkg_share, 'config', 'convoy_controller.yaml')
    planner_yaml_convoy = os.path.join(pkg_share, 'config', 'convoy_planner_server.yaml')
    recovery_yaml_convoy = os.path.join(pkg_share, 'config', 'convoy_recovery.yaml')
    bt_navigator_yaml_convoy = os.path.join(pkg_share, 'config', 'convoy_bt_navigator.yaml')

    # fc1 nav2 파일 경로
    amcl_yaml_fc1 = os.path.join(pkg_share, 'config', 'fc1_amcl_config.yaml')
    controller_yaml_fc1 = os.path.join(pkg_share, 'config', 'fc1_controller.yaml')
    planner_yaml_fc1 = os.path.join(pkg_share, 'config', 'fc1_planner_server.yaml')
    recovery_yaml_fc1 = os.path.join(pkg_share, 'config', 'fc1_recovery.yaml')
    bt_navigator_yaml_fc1 = os.path.join(pkg_share, 'config', 'fc1_bt_navigator.yaml')

    # fc2 nav2 파일 경로
    amcl_yaml_fc2 = os.path.join(pkg_share, 'config', 'fc2_amcl_config.yaml')
    controller_yaml_fc2 = os.path.join(pkg_share, 'config', 'fc2_controller.yaml')
    planner_yaml_fc2 = os.path.join(pkg_share, 'config', 'fc2_planner_server.yaml')
    recovery_yaml_fc2 = os.path.join(pkg_share, 'config', 'fc2_recovery.yaml')
    bt_navigator_yaml_fc2 = os.path.join(pkg_share, 'config', 'fc2_bt_navigator.yaml')

    # fc3 nav2 파일 경로
    amcl_yaml_fc3 = os.path.join(pkg_share, 'config', 'fc3_amcl_config.yaml')
    controller_yaml_fc3 = os.path.join(pkg_share, 'config', 'fc3_controller.yaml')
    planner_yaml_fc3 = os.path.join(pkg_share, 'config', 'fc3_planner_server.yaml')
    recovery_yaml_fc3 = os.path.join(pkg_share, 'config', 'fc3_recovery.yaml')
    bt_navigator_yaml_fc3 = os.path.join(pkg_share, 'config', 'fc3_bt_navigator.yaml')

    # rviz 파일 경로
    rviz_config_file_path = os.path.join(pkg_share, 'rviz', 'convoy_and_fc1.rviz')
    # map 파일 경로
    map_yaml_path = os.path.join(pkg_share, 'maps', 'map.yaml')


    # gazebo 모델 경로
    gazebo_models_path = os.path.join(pkg_share, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    use_sim_time = True
    fc1 = LaunchConfiguration('fc1', default='fc1')
    fc2 = LaunchConfiguration('fc2', default='fc2')
    fc3 = LaunchConfiguration('fc3', default='fc3')


    # aws warehouse world 경로
    warehouse_pkg_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    warehouse_launch_path = os.path.join(warehouse_pkg_dir, 'launch')

    # gazebo 실행
    gazebo_run = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([warehouse_launch_path, '/no_roof_small_warehouse.launch.py'])
        )

    # convoy와 follow cart gazebo에 생성
    spawn_convoy_cmd = Node(
        package='gazebo_ros',
        namespace='convoy',
        executable='spawn_entity.py',
        arguments=['-entity', convoy_name_in_model,
                   '-topic', '/convoy/robot_description',
                   '-x', convoy_spawn_x_val,
                   '-y', convoy_spawn_y_val,
                   '-z', convoy_spawn_z_val,
                   '-Y', convoy_spawn_yaw_val],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'frame_prefix': 'convoy/'}],
        remappings=[
            ("/tf", "tf")])

    spawn_fc1_cmd = Node(
        package='gazebo_ros',
        namespace='fc1',
        executable='spawn_entity.py',
        arguments=['-entity', fc1_name_in_model,
                   '-topic', '/fc1/robot_description',
                   '-x', fc1_spawn_x_val,
                   '-y', fc1_spawn_y_val,
                   '-z', fc1_spawn_z_val,
                   '-Y', fc1_spawn_yaw_val],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'frame_prefix': 'fc1/'}],
        remappings=[
            ("/tf", "tf")])

    spawn_fc2_cmd = Node(
        package='gazebo_ros',
        namespace='fc2',
        executable='spawn_entity.py',
        arguments=['-entity', fc2_name_in_model,
                   '-topic', '/fc2/robot_description',
                   '-x', fc2_spawn_x_val,
                   '-y', fc2_spawn_y_val,
                   '-z', fc2_spawn_z_val,
                   '-Y', fc2_spawn_yaw_val],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'frame_prefix': 'fc2/'}],
        remappings=[
            ("/tf", "tf")])

    spawn_fc3_cmd = Node(
        package='gazebo_ros',
        namespace='fc3',
        executable='spawn_entity.py',
        arguments=['-entity', fc3_name_in_model,
                   '-topic', '/fc3/robot_description',
                   '-x', fc3_spawn_x_val,
                   '-y', fc3_spawn_y_val,
                   '-z', fc3_spawn_z_val,
                   '-Y', fc3_spawn_yaw_val],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'frame_prefix': 'fc3/'}],
        remappings=[
            ("/tf", "tf")])

    # ekf_filter_node 실행
    # odometry, imu 정보를 센서 퓨전하여 위치 추
    convoy_localization_cmd = Node(
        package='robot_localization',
        namespace='convoy',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[localization_yaml_convoy,
                    {'use_sim_time': use_sim_time}],
        remappings=[
        ("/tf", "/convoy/tf"),
        ("/tf_static", "/convoy/tf_static"),
        ("/odom", "/convoy/odom"),
        ("/imu", "/convoy/imu"),
        ("/odometry/filtered", "/convoy/odometry/filtered")])

    fc1_localization_cmd = Node(
        package='robot_localization',
        namespace='fc1',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[localization_yaml_fc1,
                    {'use_sim_time': use_sim_time}],
        remappings=[
        ("/tf", "/fc1/tf"),
        ("/tf_static", "/fc1/tf_static"),
        ("/odom", "/fc1/odom"),
        ("/imu", "/fc1/imu"),
        ("/odometry/filtered", "/fc1/odometry/filtered")])

    fc2_localization_cmd = Node(
        package='robot_localization',
        namespace='fc2',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[localization_yaml_fc2,
                    {'use_sim_time': use_sim_time}],
        remappings=[
        ("/tf", "/fc2/tf"),
        ("/tf_static", "/fc2/tf_static"),
        ("/odom", "/fc2/odom"),
        ("/imu", "/fc2/imu"),
        ("/odometry/filtered", "/fc2/odometry/filtered")])

    fc3_localization_cmd = Node(
        package='robot_localization',
        namespace='fc3',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[localization_yaml_fc3,
                    {'use_sim_time': use_sim_time}],
        remappings=[
        ("/tf", "/fc3/tf"),
        ("/tf_static", "/fc3/tf_static"),
        ("/odom", "/fc3/odom"),
        ("/imu", "/fc3/imu"),
        ("/odometry/filtered", "/fc3/odometry/filtered")])

    # robot_state_publisher 실행
    # 로봇의 상태를 지속적으로 전달
    convoy_state_publisher_cmd = Node(
        package='robot_state_publisher',
        namespace='convoy',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', convoy_urdf_path]),
                     'frame_prefix': 'convoy/'
                     }],
        remappings=[
            ("/tf", "/convoy/tf"),
            ("/tf_static", "/convoy/tf_static"),
            ("/robot_description", "/convoy/robot_description"),
            ("/joint_states", "/convoy/joint_states")])

    fc1_state_publisher_cmd = Node(
        package='robot_state_publisher',
        namespace='fc1',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', fc_urdf_path, ' robot_name:=', fc1]),
                     'frame_prefix': 'fc1/'}],
        remappings=[
            ("/tf", "/fc1/tf"),
            ("/tf_static", "/fc1/tf_static"),
            ("/robot_description", "/fc1/robot_description"),
            ("/joint_states", "/fc1/joint_states")])

    fc2_state_publisher_cmd = Node(
        package='robot_state_publisher',
        namespace='fc2',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', fc_urdf_path, ' robot_name:=', fc2]),
                     'frame_prefix': 'fc2/'}],
        remappings=[
            ("/tf", "/fc2/tf"),
            ("/tf_static", "/fc2/tf_static"),
            ("/robot_description", "/fc2/robot_description"),
            ("/joint_states", "/fc2/joint_states")])

    fc3_state_publisher_cmd = Node(
        package='robot_state_publisher',
        namespace='fc3',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', fc_urdf_path, ' robot_name:=', fc3]),
                     'frame_prefix': 'fc3/'}],
        remappings=[
            ("/tf", "/fc3/tf"),
            ("/tf_static", "/fc3/tf_static"),
            ("/robot_description", "/fc3/robot_description"),
            ("/joint_states", "/fc3/joint_states")])

    # convoy_joint_state_publisher_cmd = Node(
    #     package='joint_state_publisher',
    #     namespace='convoy',
    #     executable='joint_state_publisher',
    #     name='convoy_joint_state_publisher',
    #     parameters=[{'use_sim_time': use_sim_time,
    #                  'robot_description': Command(['xacro ', convoy_urdf_file_path])}],
    #     remappings=[("/joint_states", "/convoy/joint_states"),
    #                 ("/robot_description", "/robot_description")])
    #
    # fc1_joint_state_publisher_cmd = Node(
    #     package='joint_state_publisher',
    #     namespace='fc1',
    #     executable='joint_state_publisher',
    #     name='fc1_joint_state_publisher',
    #     parameters=[{'use_sim_time': use_sim_time,
    #                  'robot_description': Command(['xacro ', fc_urdf_file_path, ' robot_name:=', 'fc1'])}],
    #     remappings=[("/joint_states", "/fc1/joint_states"),
    #                 ("/robot_description", "/robot_description")])
    #
    # fc2_joint_state_publisher_cmd = Node(
    #     package='joint_state_publisher',
    #     namespace='fc2',
    #     executable='joint_state_publisher',
    #     name='fc2_joint_state_publisher',
    #     parameters=[{'use_sim_time': use_sim_time,
    #                  'robot_description': Command(['xacro ', fc_urdf_file_path, ' robot_name:=', 'fc2'])}],
    #     remappings=[("/joint_states", "/fc2/joint_states"),
    #                 ("/robot_description", "/robot_description")])
    #
    # fc3_joint_state_publisher_cmd = Node(
    #     package='joint_state_publisher',
    #     namespace='fc3',
    #     executable='joint_state_publisher',
    #     name='fc3_joint_state_publisher',
    #     parameters=[{'use_sim_time': use_sim_time,
    #                  'robot_description': Command(['xacro ', fc_urdf_file_path, ' robot_name:=', 'fc3'])}],
    #     remappings=[("/joint_states", "/fc3/joint_states"),
    #                 ("/robot_description", "/robot_description")])

    # Launch the ROS 2 Navigation Stack
    # convoy_navigation = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
    #     launch_arguments={'namespace': convoy_namespace,
    #                       'use_namespace': use_namespace,
    #                       'slam': slam,
    #                       'map': map_yaml,
    #                       'use_sim_time': use_sim_time,
    #                       'params_file': convoy_params_file,
    #                       'autostart': autostart}.items())
    #
    # fc1_navigation = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
    #     launch_arguments={'namespace': fc1_namespace,
    #                       'use_namespace': use_namespace,
    #                       'slam': slam,
    #                       'map': map_yaml,
    #                       'use_sim_time': use_sim_time,
    #                       'params_file': fc1_params_file,
    #                       'autostart': autostart}.items())

    # amcl 실행
    # 위치 추정
    convoy_amcl = Node(
            namespace='convoy',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_yaml_convoy],
            remappings=[("/tf", "/convoy/tf"),
                        ("/tf_static", "/convoy/tf_static"),
                        ("/scan", "/convoy/scan"),
                        ("/map", "/map")])

    fc1_amcl = Node(
        namespace='fc1',
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_yaml_fc1],
        remappings=[("/tf", "/fc1/tf"),
                    ("/tf_static", "/fc1/tf_static"),
                    ("/scan", "/fc1/scan"),
                    ("/map", "/map")
                    ])

    fc2_amcl = Node(
        namespace='fc2',
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_yaml_fc2],
        remappings=[("/tf", "/fc2/tf"),
                    ("/tf_static", "/fc2/tf_static"),
                    ("/scan", "/fc2/scan"),
                    ("/map", "/map")
                    ])

    fc3_amcl = Node(
        namespace='fc3',
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_yaml_fc3],
        remappings=[("/tf", "/fc3/tf"),
                    ("/tf_static", "/fc3/tf_static"),
                    ("/scan", "/fc3/scan"),
                    ("/map", "/map")
                    ])

    # controller 실행
    # global path를 따라 장애물 회피 등의 local path 생성, 로봇들의 주행을 직접 제어
    convoy_controller_server = Node(
        namespace='convoy',
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml_convoy],
        remappings=[("/tf", "/convoy/tf"),
                    ("/tf_static", "/convoy/tf_static"),
                    ("/scan", "/convoy/scan"),
                    ("/map", "/map")])

    # planner 실행
    # 목적지 까지의 최적의 경로 생성
    convoy_planner_server = Node(
        namespace='convoy',
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_yaml_convoy],
        remappings=[("/tf", "/convoy/tf"),
                    ("/tf_static", "/convoy/tf_static"),
                    ("/scan", "/convoy/scan"),
                    ("/map", "/map")])

    # behavior_server 실행
    # 로봇이 경로를 찾지 못하거나 제대로 동작을 수행하지 못할 시 상태 회복을 위한 명령 실행
    convoy_recoveries_server = Node(
        namespace='convoy',
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[recovery_yaml_convoy],
        output='screen',
        remappings=[("/tf", "/convoy/tf"),
                    ("/tf_static", "/convoy/tf_static"),
                    ("/map", "/map")])

    # bt_navigator 실행
    # behavior_tree에 따라 contorller 등을 사용하여 navigation 수행
    convoy_bt_navigator = Node(
        namespace='convoy',
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml_convoy],
        remappings=[("/tf", "/convoy/tf"),
                    ("/tf_static", "/convoy/tf_static"),
                    ("/map", "/map"),
        ("/odometry/filtered", "/convoy/odometry/filtered")])

    fc1_controller_server = Node(
        namespace='fc1',
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml_fc1],
        remappings=[("/tf", "/fc1/tf"),
                    ("/tf_static", "/fc1/tf_static"),
                    ("/scan", "/fc1/scan"),
                    ("/map", "/map")
                    ])

    fc1_planner_server = Node(
        namespace='fc1',
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_yaml_fc1],
        remappings=[("/tf", "/fc1/tf"),
                    ("/tf_static", "/fc1/tf_static"),
                    ("/scan", "/fc1/scan"),
                    ("/map", "/map")
                    ])

    fc1_recoveries_server = Node(
        namespace='fc1',
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[recovery_yaml_fc1],
        output='screen',
        remappings=[("/tf", "/fc1/tf"),
                    ("/tf_static", "/fc1/tf_static"),
                    ("/map", "/map")
                    ])

    fc1_bt_navigator = Node(
        namespace='fc1',
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml_fc1],
        remappings=[("/tf", "/fc1/tf"),
                    ("/tf_static", "/fc1/tf_static"),
                    ("/map", "/map"),
        ("/odometry/filtered", "/fc1/odometry/filtered")])

    fc2_controller_server = Node(
        namespace='fc2',
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml_fc2],
        remappings=[("/tf", "/fc2/tf"),
                    ("/tf_static", "/fc2/tf_static"),
                    ("/scan", "/fc2/scan"),
                    ("/map", "/map")
                    ])

    fc2_planner_server = Node(
        namespace='fc2',
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_yaml_fc2],
        remappings=[("/tf", "/fc2/tf"),
                    ("/tf_static", "/fc2/tf_static"),
                    ("/scan", "/fc2/scan"),
                    ("/map", "/map")
                    ])

    fc2_recoveries_server = Node(
        namespace='fc2',
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[recovery_yaml_fc2],
        output='screen',
        remappings=[("/tf", "/fc2/tf"),
                    ("/tf_static", "/fc2/tf_static"),
                    ("/map", "/map")
                    ])

    fc2_bt_navigator = Node(
        namespace='fc2',
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml_fc2],
        remappings=[("/tf", "/fc2/tf"),
                    ("/tf_static", "/fc2/tf_static"),
                    ("/map", "/map"),
        ("/odometry/filtered", "/fc2/odometry/filtered")])

    fc3_controller_server = Node(
        namespace='fc3',
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml_fc3],
        remappings=[("/tf", "/fc3/tf"),
                    ("/tf_static", "/fc3/tf_static"),
                    ("/scan", "/fc3/scan"),
                    ("/map", "/map")
                    ])

    fc3_planner_server = Node(
        namespace='fc3',
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_yaml_fc3],
        remappings=[("/tf", "/fc3/tf"),
                    ("/tf_static", "/fc3/tf_static"),
                    ("/scan", "/fc3/scan"),
                    ("/map", "/map")
                    ])

    fc3_recoveries_server = Node(
        namespace='fc3',
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[recovery_yaml_fc3],
        output='screen',
        remappings=[("/tf", "/fc3/tf"),
                    ("/tf_static", "/fc3/tf_static"),
                    ("/map", "/map")
                    ])

    fc3_bt_navigator = Node(
        namespace='fc3',
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml_fc3],
        remappings=[("/tf", "/fc3/tf"),
                    ("/tf_static", "/fc3/tf_static"),
                    ("/map", "/map"),
        ("/odometry/filtered", "/fc3/odometry/filtered")])

    # lifecycle_manager 실행
    # navigation을 위한 노드들을 관리
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
                    {'autostart': True},
                    {'bond_timeout': 0.0},
                    {'node_names': ['map_server',
                                    'convoy/amcl',
                                    'fc1/amcl',
                                    'fc2/amcl',
                                    'fc3/amcl',
                                    'convoy/planner_server',
                                    'convoy/controller_server',
                                    'convoy/behavior_server',
                                    'convoy/bt_navigator',
                                    'fc1/planner_server',
                                    'fc1/controller_server',
                                    'fc1/behavior_server',
                                    'fc1/bt_navigator',
                                    'fc2/planner_server',
                                    'fc2/controller_server',
                                    'fc2/behavior_server',
                                    'fc2/bt_navigator',
                                    'fc3/planner_server',
                                    'fc3/controller_server',
                                    'fc3/behavior_server',
                                    'fc3/bt_navigator'
                                    ]}])

    # lifecycle_manager_localization = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_localization',
    #     output='screen',
    #     parameters=[{'use_sim_time': True},
    #                 {'autostart': True},
    #                 {'bond_timeout': 0.0},
    #                 {'node_names': ['map_server',
    #                                 'convoy/amcl',
    #                                 'fc1/amcl',
    #                                 'fc2/amcl',
    #                                 'fc3/amcl',
    #                                 ]}])
    #
    # lifecycle_manager_pathplanner = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_pathplanner',
    #     output='screen',
    #     parameters=[{'use_sim_time': True},
    #                 {'autostart': True},
    #                 {'bond_timeout': 0.0},
    #                 {'node_names': [
    #                                 'convoy/planner_server',
    #                                 'convoy/controller_server',
    #                                 'convoy/behavior_server',
    #                                 'convoy/bt_navigator',
    #                                 'fc1/planner_server',
    #                                 'fc1/controller_server',
    #                                 'fc1/behavior_server',
    #                                 'fc1/bt_navigator',
    #                                 'fc2/planner_server',
    #                                 'fc2/controller_server',
    #                                 'fc2/behavior_server',
    #                                 'fc2/bt_navigator',
    #                                 'fc3/planner_server',
    #                                 'fc3/controller_server',
    #                                 'fc3/behavior_server',
    #                                 'fc3/bt_navigator'
    #                                 ]}])

    # map_server 실행
    # map 정보를 전달
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'topic_name': "/map"},
                    {'frame_id': "map"},
                    {'yaml_filename': map_yaml_path}])

    # rviz 실행
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file_path]
    )

    # dial을 통해 convoy를 제어
    convoy_controller = Node(
        package='follow_cart',
        executable='convoy_controller',
        name='convoy_controller',
        output='screen'
    )

    follower_controller = Node(
        package='follow_cart',
        executable='follower_controller',
        name='follower_controller',
        output='screen'
    )

    # launch description 생성
    ld = LaunchDescription()

    # launch description에 노드들 추가
    # 노드가 실행될 수 있게
    ld.add_action(gazebo_run)
    ld.add_action(spawn_convoy_cmd)
    ld.add_action(spawn_fc1_cmd)
    ld.add_action(spawn_fc2_cmd)
    ld.add_action(spawn_fc3_cmd)

    ld.add_action(convoy_localization_cmd)
    ld.add_action(fc1_localization_cmd)
    ld.add_action(fc2_localization_cmd)
    ld.add_action(fc3_localization_cmd)

    ld.add_action(convoy_state_publisher_cmd)
    ld.add_action(fc1_state_publisher_cmd)
    ld.add_action(fc2_state_publisher_cmd)
    ld.add_action(fc3_state_publisher_cmd)

    # ld.add_action(convoy_joint_state_publisher_cmd)
    # ld.add_action(fc1_joint_state_publisher_cmd)
    # ld.add_action(fc2_joint_state_publisher_cmd)
    # ld.add_action(fc3_joint_state_publisher_cmd)


    # ld.add_action(lifecycle_manager_localization)
    # ld.add_action(lifecycle_manager_pathplanner)
    ld.add_action(lifecycle_manager)
    ld.add_action(map_server)

    ld.add_action(convoy_amcl)
    ld.add_action(fc1_amcl)
    ld.add_action(fc2_amcl)
    ld.add_action(fc3_amcl)

    ld.add_action(convoy_controller_server)
    ld.add_action(convoy_planner_server)
    ld.add_action(convoy_recoveries_server)
    ld.add_action(convoy_bt_navigator)

    ld.add_action(fc1_controller_server)
    ld.add_action(fc1_planner_server)
    ld.add_action(fc1_recoveries_server)
    ld.add_action(fc1_bt_navigator)

    ld.add_action(fc2_controller_server)
    ld.add_action(fc2_planner_server)
    ld.add_action(fc2_recoveries_server)
    ld.add_action(fc2_bt_navigator)

    ld.add_action(fc3_controller_server)
    ld.add_action(fc3_planner_server)
    ld.add_action(fc3_recoveries_server)
    ld.add_action(fc3_bt_navigator)

    ld.add_action(rviz)

    ld.add_action(convoy_controller)
    # ld.add_action(follower_controller)

    return ld