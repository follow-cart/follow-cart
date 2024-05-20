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

    # 로봇 초기 생성 위치 !고정!
    convoy_spawn_x_val = '0.0'
    convoy_spawn_y_val = '-2.0'
    convoy_spawn_z_val = '0.1'
    convoy_spawn_yaw_val = '0.0'

    # convoy yaw: 0.0 원점 기준
    # 세로 대형: -1.0 가로 대형: -1.0 삼각 대형: -1.7
    fc1_spawn_x_val = '-1.7'
    # 세로 대형: 0.0 가로 대형: -1.0 삼각 대형: -1.0
    fc1_spawn_y_val = '-1.0'
    fc1_spawn_z_val = '0.1'
    fc1_spawn_yaw_val = '0.0'

    # 세로 대형: -2.0 가로 대형: -1.0 삼각 대형: -1.0
    fc2_spawn_x_val = '-1.0'
    # 세로 대형: 0.0 가로 대형: 0.0 삼각 대형: 0.0
    fc2_spawn_y_val = '-2.0'
    fc2_spawn_z_val = '0.1'
    fc2_spawn_yaw_val = '0.0'

    # 세로 대형: -3.0 가로 대형: -1.0 삼각 대형: -1.7
    fc3_spawn_x_val = '-1.7'
    # 세로 대형: 0.0 가로 대형: +1.0 삼각 대형: +1.0
    fc3_spawn_y_val = '-3.0'
    fc3_spawn_z_val = '0.1'
    fc3_spawn_yaw_val = '0.0'

    # urdf 파일 경로
    # pkg_share = FindPackageShare(package=package_name).find(package_name)
    pkg_share = get_package_share_directory(package_name)
    # convoy_urdf_path = os.path.join(pkg_share, 'urdf', 'convoy', 'jetbot.urdf')
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
    convoy_rviz_config_file_path = os.path.join(pkg_share, 'rviz', 'convoy.rviz')
    fc1_rviz_config_file_path = os.path.join(pkg_share, 'rviz', 'fc1.rviz')
    fc2_rviz_config_file_path = os.path.join(pkg_share, 'rviz', 'fc2.rviz')
    fc3_rviz_config_file_path = os.path.join(pkg_share, 'rviz', 'fc3.rviz')

    # map 파일 경로
    convoy_map_yaml_path = os.path.join(pkg_share, 'maps', 'convoy_map.yaml')
    fc1_map_yaml_path = os.path.join(pkg_share, 'maps', 'fc1_map.yaml')
    fc2_map_yaml_path = os.path.join(pkg_share, 'maps', 'fc2_map.yaml')
    fc3_map_yaml_path = os.path.join(pkg_share, 'maps', 'fc3_map.yaml')


    # gazebo 모델 경로
    gazebo_models_path = os.path.join(pkg_share, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    # pedestrian 파일 경로
    pedestrian_sdf_path = os.path.join(pkg_share, 'models', 'pedestrian', 'model.sdf')

    use_sim_time = True
    convoy = LaunchConfiguration('convoy', default='convoy')
    fc1 = LaunchConfiguration('fc1', default='fc1')
    fc2 = LaunchConfiguration('fc2', default='fc2')
    fc3 = LaunchConfiguration('fc3', default='fc3')


    # aws warehouse world 경로
    warehouse_pkg_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    warehouse_launch_path = os.path.join(warehouse_pkg_dir, 'launch')

    # gazebo 실행
    gazebo_run = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([warehouse_launch_path, '/small_warehouse.launch.py'])
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
        parameters=[{'use_sim_time': use_sim_time}],
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
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ("/tf", "tf")])

    spawn_pedestrian_cmd = Node(
        package='gazebo_ros',
        namespace='pedestrian',
        executable='spawn_entity.py',
        name="spawn_pedestrian",
        arguments=['-entity', 'pedestrian',
                   '-file', pedestrian_sdf_path,
                   '-x', '1.5',
                   '-y', '1.5',
                   '-z', '0.1'],
        output='screen')

    pedestrian_controller_cmd = Node(
        package='follow_cart',
        namespace='pedestrian',
        executable='pedestrian_controller',
        name="pedestrian_controller",
        output='screen')

    # 프로세스 처리
    convoy_camera_cmd = Node(
        package='follow_cart',
        namespace='convoy',
        executable='pedestrian_detect_processor',
        name="pedestrian_detect_processor",
        output='screen'
    )

    # 카메라 영상 처리
    display_image_cmd = Node(
        package='follow_cart',
        namespace='convoy',
        executable='pedestrian_detector',
        name='pedestrian_detector',
        output='screen'
    )



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
        ("/map", "map"),
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
        ("/map", "map"),
        ("/odometry/filtered", "odometry/filtered"),
        ("/amcl_pose", "amcl_pose")])

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
        ("/tf_static", "tf_static"),
        ("/odom", "odom"),
        ("/imu", "imu"),
        ("/map", "map"),
        ("/odometry/filtered", "odometry/filtered")])

    fc3_localization_cmd = Node(
        package='robot_localization',
        namespace='fc3',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[localization_yaml_fc3,
                    {'use_sim_time': use_sim_time}],
        remappings=[
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
        ("/odom", "odom"),
        ("/imu", "imu"),
        ("/map", "map"),
        ("/odometry/filtered", "odometry/filtered")])

    # robot_state_publisher 실행
    # 로봇의 상태를 지속적으로 전달
    convoy_state_publisher_cmd = Node(
        package='robot_state_publisher',
        namespace='convoy',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', fc_urdf_path, ' robot_name:=', convoy])
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

    fc2_state_publisher_cmd = Node(
        package='robot_state_publisher',
        namespace='fc2',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', fc_urdf_path, ' robot_name:=', fc2])}],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("/robot_description", "robot_description")])

    fc3_state_publisher_cmd = Node(
        package='robot_state_publisher',
        namespace='fc3',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', fc_urdf_path, ' robot_name:=', fc3])}],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("/robot_description", "robot_description")])



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
                        ("/map", "map"),
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
                    ("/map", "map"),
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
                    ("/map", "map"),
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
                    ("/map", "map"),
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
                    ("/map", "map"),
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
                    ("/map", "map"),
                    ("/odometry/filtered", "odometry/filtered")])

    fc2_amcl = Node(
        namespace='fc2',
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_yaml_fc2],
        remappings=[("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                    ("/scan", "scan"),
                    ("/map", "map"),
                    ("/amcl_pose", "amcl_pose")])

    fc2_controller_server = Node(
        namespace='fc2',
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml_fc2],
        remappings=[("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                    ("/scan", "scan")])

    fc2_planner_server = Node(
        namespace='fc2',
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_yaml_fc2],
        remappings=[("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                    ("/map", "map"),
                    ("/scan", "scan")])

    fc2_recoveries_server = Node(
        namespace='fc2',
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[recovery_yaml_fc2],
        output='screen',
        remappings=[("/tf", "tf"),
                    ("/tf_static", "tf_static")])

    fc2_bt_navigator = Node(
        namespace='fc2',
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml_fc2],
        remappings=[("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                    ("/map", "map"),
                    ("/odometry/filtered", "odometry/filtered")])

    fc3_amcl = Node(
        namespace='fc3',
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_yaml_fc3],
        remappings=[("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                    ("/scan", "scan"),
                    ("/map", "map"),
                    ("/amcl_pose", "amcl_pose")])

    fc3_controller_server = Node(
        namespace='fc3',
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml_fc3],
        remappings=[("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                    ("/scan", "scan")])

    fc3_planner_server = Node(
        namespace='fc3',
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_yaml_fc3],
        remappings=[("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                    ("/map", "map"),
                    ("/scan", "scan")])

    fc3_recoveries_server = Node(
        namespace='fc3',
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[recovery_yaml_fc3],
        output='screen',
        remappings=[("/tf", "tf"),
                    ("/tf_static", "tf_static")])

    fc3_bt_navigator = Node(
        namespace='fc3',
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml_fc3],
        remappings=[("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                    ("/map", "map"),
                    ("/odometry/filtered", "odometry/filtered")])

    # lifecycle_manager 실행
    # navigation을 위한 노드들을 관리
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'bond_timeout': 0.0},
                    {'node_names': ['convoy/map_server',
                                    'convoy/amcl',
                                    'fc1/map_server',
                                    'fc1/amcl',
                                    'fc2/map_server',
                                    'fc2/amcl',
                                    # 'fc3/map_server',
                                    # 'fc3/amcl'
                                    ]}])

    lifecycle_manager_path_planning = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_path_planning',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'bond_timeout': 0.0},
                    {'node_names': ['convoy/controller_server',
                                    'convoy/planner_server',
                                    'convoy/behavior_server',
                                    'convoy/bt_navigator',
                                    'fc1/controller_server',
                                    'fc1/planner_server',
                                    'fc1/behavior_server',
                                    'fc1/bt_navigator',
                                    'fc2/controller_server',
                                    'fc2/planner_server',
                                    'fc2/behavior_server',
                                    'fc2/bt_navigator',
                                    # 'fc3/controller_server',
                                    # 'fc3/planner_server',
                                    # 'fc3/behavior_server',
                                    # 'fc3/bt_navigator'
                                    ]}])


    # fc1_lifecycle_manager = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='fc1_lifecycle_manager',
    #     output='screen',
    #     parameters=[{'use_sim_time': True},
    #                 {'autostart': True},
    #                 {'bond_timeout': 0.0},
    #                 {'node_names': [
    #
    #                                 ]}])

    # fc2_lifecycle_manager = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='fc2_lifecycle_manager',
    #     output='screen',
    #     parameters=[{'use_sim_time': True},
    #                 {'autostart': True},
    #                 {'bond_timeout': 0.0},
    #                 {'node_names': [
    #
    #                 ]}])
    #
    # fc3_lifecycle_manager = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='fc3_lifecycle_manager',
    #     output='screen',
    #     parameters=[{'use_sim_time': True},
    #                 {'autostart': True},
    #                 {'bond_timeout': 0.0},
    #                 {'node_names': [
    #
    #                 ]}])


    # map_server 실행
    # map 정보를 전달
    convoy_map_server = Node(
        package='nav2_map_server',
        namespace='convoy',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'topic_name': "/convoy/map"},
                    {'frame_id': "convoy/map"},
                    {'yaml_filename': convoy_map_yaml_path}])

    fc1_map_server = Node(
        package='nav2_map_server',
        namespace='fc1',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'topic_name': "/fc1/map"},
                    {'frame_id': "fc1/map"},
                    {'yaml_filename': fc1_map_yaml_path}])

    fc2_map_server = Node(
        package='nav2_map_server',
        namespace='fc2',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'topic_name': "/fc2/map"},
                    {'frame_id': "fc2/map"},
                    {'yaml_filename': fc2_map_yaml_path}])

    fc3_map_server = Node(
        package='nav2_map_server',
        namespace='fc3',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'topic_name': "/fc3/map"},
                    {'frame_id': "fc3/map"},
                    {'yaml_filename': fc3_map_yaml_path}])

    # rviz 실행
    convoy_rviz = Node(
        package='rviz2',
        namespace='convoy',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', convoy_rviz_config_file_path],
        remappings=[("/tf", "/convoy/tf"),
                    ("/tf_static", "/convoy/tf_static"),
                    ("/scan", "/convoy/scan"),
                    ("/odom", "/convoy/odom"),
                    ("/amcl_pose", "/convoy/amcl_pose"),
                    ("/map", "/convoy/map"),
                    ("/robot_description", "/convoy/robot_description")])

    fc1_rviz = Node(
        package='rviz2',
        namespace='fc1',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', fc1_rviz_config_file_path],
        remappings=[("/tf", "/fc1/tf"),
                    ("/tf_static", "/fc1/tf_static"),
                    ("/scan", "/fc1/scan"),
                    ("/odom", "/fc1/odom"),
                    ("/amcl_pose", "/fc1/amcl_pose"),
                    ("/map", "/fc1/map"),
                    ("/robot_description", "/fc1/robot_description")])

    fc2_rviz = Node(
        package='rviz2',
        namespace='fc2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', fc2_rviz_config_file_path],
        remappings=[("/tf", "/fc2/tf"),
                    ("/tf_static", "/fc2/tf_static"),
                    ("/scan", "/fc2/scan"),
                    ("/odom", "/fc2/odom"),
                    ("/amcl_pose", "/fc2/amcl_pose"),
                    ("/map", "/fc2/map"),
                    ("/robot_description", "/fc2/robot_description")])

    fc3_rviz = Node(
        package='rviz2',
        namespace='fc3',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', fc3_rviz_config_file_path],
        remappings=[("/tf", "/fc3/tf"),
                    ("/tf_static", "/fc3/tf_static"),
                    ("/scan", "/fc3/scan"),
                    ("/odom", "/fc3/odom"),
                    ("/amcl_pose", "/fc3/amcl_pose"),
                    ("/map", "/fc3/map"),
                    ("/robot_description", "/fc3/robot_description")])


    # 대형을 유지하며 convoy를 따라가게 하는 fc1 controller
    fc1_controller = Node(
        package='follow_cart',
        executable='fc1_controller',
        name='fc1_controller',
        output='screen'
    )

    fc1_goal_updater = Node(
        package='follow_cart',
        executable='fc1_goal_updater',
        name='fc1_goal_updater',
        output='screen'
    )

    fc2_controller = Node(
        package='follow_cart',
        executable='fc2_controller',
        name='fc2_controller',
        output='screen'
    )

    fc2_goal_updater = Node(
        package='follow_cart',
        executable='fc2_goal_updater',
        name='fc2_goal_updater',
        output='screen'
    )

    fc3_controller = Node(
        package='follow_cart',
        executable='fc3_controller',
        name='fc3_controller',
        output='screen'
    )

    fc3_goal_updater = Node(
        package='follow_cart',
        executable='fc3_goal_updater',
        name='fc3_goal_updater',
        output='screen'
    )

    # launch description 생성
    ld = LaunchDescription()

    # launch description에 노드들 추가
    # 노드가 실행될 수 있게
    ld.add_action(gazebo_run)

    ld.add_action(spawn_convoy_cmd)
    ld.add_action(convoy_state_publisher_cmd)
    ld.add_action(convoy_localization_cmd)

    # ld.add_action(spawn_fc1_cmd)
    # ld.add_action(fc1_state_publisher_cmd)
    # ld.add_action(fc1_localization_cmd)


    # ld.add_action(spawn_fc2_cmd)
    # ld.add_action(fc2_state_publisher_cmd)
    # ld.add_action(fc2_localization_cmd)

    # ld.add_action(spawn_fc3_cmd)
    # ld.add_action(fc3_state_publisher_cmd)
    # ld.add_action(fc3_localization_cmd)

    # 보행자
    ld.add_action(spawn_pedestrian_cmd)
    ld.add_action(pedestrian_controller_cmd)

    # convoy 카메라
    ld.add_action(convoy_camera_cmd)
    ld.add_action(display_image_cmd)

    # ld.add_action(convoy_map_server)
    # ld.add_action(convoy_amcl)
    # ld.add_action(convoy_controller_server)
    # ld.add_action(convoy_planner_server)
    # ld.add_action(convoy_recoveries_server)
    # ld.add_action(convoy_bt_navigator)

    # ld.add_action(fc1_map_server)
    # ld.add_action(fc1_amcl)
    # ld.add_action(fc1_controller_server)
    # ld.add_action(fc1_planner_server)
    # ld.add_action(fc1_recoveries_server)
    # ld.add_action(fc1_bt_navigator)
    #
    # ld.add_action(fc2_map_server)
    # ld.add_action(fc2_amcl)
    # ld.add_action(fc2_controller_server)
    # ld.add_action(fc2_planner_server)
    # ld.add_action(fc2_recoveries_server)
    # ld.add_action(fc2_bt_navigator)

    # ld.add_action(fc3_map_server)
    # ld.add_action(fc3_amcl)
    # ld.add_action(fc3_controller_server)
    # ld.add_action(fc3_planner_server)
    # ld.add_action(fc3_recoveries_server)
    # ld.add_action(fc3_bt_navigator)

    # ld.add_action(lifecycle_manager_localization)
    # ld.add_action(lifecycle_manager_path_planning)

    # ld.add_action(fc1_controller)
    # ld.add_action(fc1_goal_updater)
    #
    # ld.add_action(fc2_controller)
    # ld.add_action(fc2_goal_updater)

    # ld.add_action(fc3_controller)
    # ld.add_action(fc3_goal_updater)

    ld.add_action(convoy_rviz)
    # ld.add_action(fc1_rviz)
    # ld.add_action(fc2_rviz)
    # ld.add_action(fc3_rviz)

    return ld