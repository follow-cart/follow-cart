import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    package_name = 'follow_cart'
    convoy_name_in_model = 'convoy'
    fc1_name_in_model = 'follow_cart_1'
    fc2_name_in_model = 'follow_cart_2'
    fc3_name_in_model = 'follow_cart_3'

    # 로봇 초기 생성 위치
    convoy_spawn_x_val = '0.0'
    convoy_spawn_y_val = '0.0'
    convoy_spawn_z_val = '0.25'
    convoy_spawn_yaw_val = '0.0'

    fc1_spawn_x_val = '-1.0'
    fc1_spawn_y_val = '0.0'
    fc1_spawn_z_val = '0.25'
    fc1_spawn_yaw_val = '0.0'

    fc2_spawn_x_val = '-1.0'
    fc2_spawn_y_val = '-2.5'
    fc2_spawn_z_val = '0.25'
    fc2_spawn_yaw_val = '0.0'

    fc3_spawn_x_val = '-1.0'
    fc3_spawn_y_val = '-3.5'
    fc3_spawn_z_val = '0.25'
    fc3_spawn_yaw_val = '0.0'

    # urdf 파일 경로
    pkg_share = FindPackageShare(package=package_name).find(package_name)
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
    rviz_config_file_path = os.path.join(pkg_share, 'rviz', 'multi_robot.rviz')
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
        parameters=[{'use_sim_time': use_sim_time}],
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
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ("/tf", "tf")])

    # spawn_fc2_cmd = Node(
    #     package='gazebo_ros',
    #     namespace='fc2',
    #     executable='spawn_entity.py',
    #     arguments=['-entity', fc2_name_in_model,
    #                '-topic', '/fc2/robot_description',
    #                '-x', fc2_spawn_x_val,
    #                '-y', fc2_spawn_y_val,
    #                '-z', fc2_spawn_z_val,
    #                '-Y', fc2_spawn_yaw_val],
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     remappings=[
    #         ("/tf", "tf")])
    #
    # spawn_fc3_cmd = Node(
    #     package='gazebo_ros',
    #     namespace='fc3',
    #     executable='spawn_entity.py',
    #     arguments=['-entity', fc3_name_in_model,
    #                '-topic', '/fc3/robot_description',
    #                '-x', fc3_spawn_x_val,
    #                '-y', fc3_spawn_y_val,
    #                '-z', fc3_spawn_z_val,
    #                '-Y', fc3_spawn_yaw_val],
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     remappings=[
    #         ("/tf", "tf")])

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
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
        ("/odom", "odom"),
        ("/imu", "imu"),
        ("/odometry/filtered", "odometry/filtered"),
        ("/amcl_pose", "amcl_pose")])

    fc1_localization_cmd = Node(
        package='robot_localization',
        namespace='fc1',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[localization_yaml_fc1,
                    {'use_sim_time': use_sim_time}],
        remappings=[
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
        ("/odom", "odom"),
        ("/imu", "imu"),
        ("/odometry/filtered", "odometry/filtered"),
        ("/amcl_pose", "amcl_pose")])

    # fc2_localization_cmd = Node(
    #     package='robot_localization',
    #     namespace='fc2',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[localization_yaml_fc2,
    #                 {'use_sim_time': use_sim_time}],
    #     remappings=[
    #     ("/tf", "/fc2/tf"),
    #     ("/tf_static", "tf_static"),
    #     ("/odom", "odom"),
    #     ("/imu", "imu"),
    #     ("/odometry/filtered", "odometry/filtered")])
    #
    # fc3_localization_cmd = Node(
    #     package='robot_localization',
    #     namespace='fc3',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[localization_yaml_fc3,
    #                 {'use_sim_time': use_sim_time}],
    #     remappings=[
    #     ("/tf", "tf"),
    #     ("/tf_static", "tf_static"),
    #     ("/odom", "odom"),
    #     ("/imu", "imu"),
    #     ("/odometry/filtered", "odometry/filtered")])

    # robot_state_publisher 실행
    # 로봇의 상태를 지속적으로 전달
    convoy_state_publisher_cmd = Node(
        package='robot_state_publisher',
        namespace='convoy',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', convoy_urdf_path])
                     }],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("/robot_description", "robot_description")])

    fc1_state_publisher_cmd = Node(
        package='robot_state_publisher',
        namespace='fc1',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', fc_urdf_path, ' robot_name:=', fc1])}],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("/robot_description", "robot_description")])

    # fc2_state_publisher_cmd = Node(
    #     package='robot_state_publisher',
    #     namespace='fc2',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time,
    #                  'robot_description': Command(['xacro ', fc_urdf_path, ' robot_name:=', fc2])}],
    #     remappings=[
    #         ("/tf", "tf"),
    #         ("/tf_static", "tf_static"),
    #         ("/robot_description", "robot_description")])
    #
    # fc3_state_publisher_cmd = Node(
    #     package='robot_state_publisher',
    #     namespace='fc3',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time,
    #                  'robot_description': Command(['xacro ', fc_urdf_path, ' robot_name:=', fc3])}],
    #     remappings=[
    #         ("/tf", "tf"),
    #         ("/tf_static", "tf_static"),
    #         ("/robot_description", "robot_description")])



    # amcl 실행
    # 위치 추정
    convoy_amcl = Node(
            namespace='convoy',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_yaml_convoy],
            remappings=[("/tf", "tf"),
                        ("/tf_static", "tf_static"),
                        ("/scan", "scan"),
                        ("/amcl_pose", "amcl_pose")])

    # controller 실행
    # global path를 따라 장애물 회피 등의 local path 생성, 로봇들의 주행을 직접 제어
    convoy_controller_server = Node(
        namespace='convoy',
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml_convoy],
        remappings=[("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                    ("/scan", "scan"),
                    ("/cmd_vel", "cmd_vel")])

    # planner 실행
    # 목적지 까지의 최적의 경로 생성
    convoy_planner_server = Node(
        namespace='convoy',
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_yaml_convoy],
        remappings=[("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                    ("/scan", "scan")])

    # behavior_server 실행
    # 로봇이 경로를 찾지 못하거나 제대로 동작을 수행하지 못할 시 상태 회복을 위한 명령 실행
    convoy_recoveries_server = Node(
        namespace='convoy',
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[recovery_yaml_convoy],
        output='screen',
        remappings=[("/tf", "tf"),
                    ("/tf_static", "tf_static")])

    # bt_navigator 실행
    # behavior_tree에 따라 contorller 등을 사용하여 navigation 수행
    convoy_bt_navigator = Node(
        namespace='convoy',
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml_convoy],
        remappings=[("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                    ("/odometry/filtered", "odometry/filtered")])

    fc1_amcl = Node(
        namespace='fc1',
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_yaml_fc1],
        remappings=[("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                    ("/scan", "scan"),
                    ("/amcl_pose", "amcl_pose")])

    fc1_controller_server = Node(
        namespace='fc1',
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml_fc1],
        remappings=[("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                    ("/scan", "scan")])

    fc1_planner_server = Node(
        namespace='fc1',
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_yaml_fc1],
        remappings=[("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                    ("/scan", "scan")])

    fc1_recoveries_server = Node(
        namespace='fc1',
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[recovery_yaml_fc1],
        output='screen',
        remappings=[("/tf", "tf"),
                    ("/tf_static", "tf_static")])

    fc1_bt_navigator = Node(
        namespace='fc1',
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml_fc1],
        remappings=[("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                    ("/odometry/filtered", "odometry/filtered")])

    # fc2_amcl = Node(
    #     namespace='fc2',
    #     package='nav2_amcl',
    #     executable='amcl',
    #     name='amcl',
    #     output='screen',
    #     parameters=[amcl_yaml_fc2],
    #     remappings=[("/tf", "tf"),
    #                 ("/tf_static", "tf_static"),
    #                 ("/scan", "scan")])
    #
    # fc2_controller_server = Node(
    #     namespace='fc2',
    #     package='nav2_controller',
    #     executable='controller_server',
    #     name='controller_server',
    #     output='screen',
    #     parameters=[controller_yaml_fc2],
    #     remappings=[("/tf", "tf"),
    #                 ("/tf_static", "tf_static"),
    #                 ("/scan", "scan")])
    #
    # fc2_planner_server = Node(
    #     namespace='fc2',
    #     package='nav2_planner',
    #     executable='planner_server',
    #     name='planner_server',
    #     output='screen',
    #     parameters=[planner_yaml_fc2],
    #     remappings=[("/tf", "tf"),
    #                 ("/tf_static", "tf_static"),
    #                 ("/scan", "scan")])
    #
    # fc2_recoveries_server = Node(
    #     namespace='fc2',
    #     package='nav2_behaviors',
    #     executable='behavior_server',
    #     name='behavior_server',
    #     parameters=[recovery_yaml_fc2],
    #     output='screen',
    #     remappings=[("/tf", "tf"),
    #                 ("/tf_static", "tf_static")])
    #
    # fc2_bt_navigator = Node(
    #     namespace='fc2',
    #     package='nav2_bt_navigator',
    #     executable='bt_navigator',
    #     name='bt_navigator',
    #     output='screen',
    #     parameters=[bt_navigator_yaml_fc2],
    #     remappings=[("/tf", "tf"),
    #                 ("/tf_static", "tf_static"),
    #                 ("/odometry/filtered", "odometry/filtered")])
    #
    # fc3_amcl = Node(
    #     namespace='fc3',
    #     package='nav2_amcl',
    #     executable='amcl',
    #     name='amcl',
    #     output='screen',
    #     parameters=[amcl_yaml_fc3],
    #     remappings=[("/tf", "tf"),
    #                 ("/tf_static", "tf_static"),
    #                 ("/scan", "scan")])
    #
    # fc3_controller_server = Node(
    #     namespace='fc3',
    #     package='nav2_controller',
    #     executable='controller_server',
    #     name='controller_server',
    #     output='screen',
    #     parameters=[controller_yaml_fc3],
    #     remappings=[("/tf", "tf"),
    #                 ("/tf_static", "tf_static"),
    #                 ("/scan", "scan")])
    #
    # fc3_planner_server = Node(
    #     namespace='fc3',
    #     package='nav2_planner',
    #     executable='planner_server',
    #     name='planner_server',
    #     output='screen',
    #     parameters=[planner_yaml_fc3],
    #     remappings=[("/tf", "tf"),
    #                 ("/tf_static", "tf_static"),
    #                 ("/scan", "scan")])
    #
    # fc3_recoveries_server = Node(
    #     namespace='fc3',
    #     package='nav2_behaviors',
    #     executable='behavior_server',
    #     name='behavior_server',
    #     parameters=[recovery_yaml_fc3],
    #     output='screen',
    #     remappings=[("/tf", "tf"),
    #                 ("/tf_static", "tf_static")])
    #
    # fc3_bt_navigator = Node(
    #     namespace='fc3',
    #     package='nav2_bt_navigator',
    #     executable='bt_navigator',
    #     name='bt_navigator',
    #     output='screen',
    #     parameters=[bt_navigator_yaml_fc3],
    #     remappings=[("/tf", "tf"),
    #                 ("/tf_static", "tf_static"),
    #                 ("/odometry/filtered", "odometry/filtered")])

    # lifecycle_manager 실행
    # navigation을 위한 노드들을 관리
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'bond_timeout': 0.0},
                    {'node_names': ['map_server',
                                    'convoy/amcl',
                                    'convoy/controller_server',
                                    'convoy/planner_server',
                                    'convoy/behavior_server',
                                    'convoy/bt_navigator',
                                    'fc1/amcl',
                                    'fc1/controller_server',
                                    'fc1/planner_server',
                                    'fc1/behavior_server',
                                    'fc1/bt_navigator'
                                    ]}])

    # 'fc2/amcl',
    # 'fc2/planner_server',
    # 'fc2/controller_server',
    # 'fc2/behavior_server',
    # 'fc2/bt_navigator',
    # 'fc3/amcl',
    # 'fc3/planner_server',
    # 'fc3/controller_server',
    # 'fc3/behavior_server',
    # 'fc3/bt_navigator',

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
    #                                 'fc3/bt_navigator',
    #                                 ]}])

    # map_server 실행
    # map 정보를 전달
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'topic_name': "map"},
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

    # 대형을 유지하며 convoy를 따라가게 하는 fc1 controller
    fc1_controller = Node(
        package='follow_cart',
        executable='fc1_controller',
        name='fc1_controller',
        output='screen'
    )

    # launch description 생성
    ld = LaunchDescription()

    # launch description에 노드들 추가
    # 노드가 실행될 수 있게
    ld.add_action(gazebo_run)
    ld.add_action(spawn_convoy_cmd)
    ld.add_action(spawn_fc1_cmd)
    # ld.add_action(spawn_fc2_cmd)
    # ld.add_action(spawn_fc3_cmd)

    ld.add_action(convoy_localization_cmd)
    ld.add_action(fc1_localization_cmd)
    # ld.add_action(fc2_localization_cmd)
    # ld.add_action(fc3_localization_cmd)

    ld.add_action(convoy_state_publisher_cmd)
    ld.add_action(fc1_state_publisher_cmd)
    # ld.add_action(fc2_state_publisher_cmd)
    # ld.add_action(fc3_state_publisher_cmd)

    # ld.add_action(lifecycle_manager_localization)
    # ld.add_action(lifecycle_manager_pathplanner)

    ld.add_action(map_server)

    ld.add_action(convoy_amcl)
    ld.add_action(convoy_controller_server)
    ld.add_action(convoy_planner_server)
    ld.add_action(convoy_recoveries_server)
    ld.add_action(convoy_bt_navigator)

    ld.add_action(fc1_amcl)
    ld.add_action(fc1_controller_server)
    ld.add_action(fc1_planner_server)
    ld.add_action(fc1_recoveries_server)
    ld.add_action(fc1_bt_navigator)

    # ld.add_action(fc2_amcl)
    # ld.add_action(fc2_bt_navigator)
    # ld.add_action(fc2_planner_server)
    # ld.add_action(fc2_controller_server)
    # ld.add_action(fc2_recoveries_server)
    #
    # ld.add_action(fc3_amcl)
    # ld.add_action(fc3_bt_navigator)
    # ld.add_action(fc3_planner_server)
    # ld.add_action(fc3_controller_server)
    # ld.add_action(fc3_recoveries_server)

    ld.add_action(lifecycle_manager)

    # ld.add_action(rviz)

    ld.add_action(fc1_controller)

    return ld