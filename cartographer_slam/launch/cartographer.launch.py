import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():

    package_name = 'cartographer_slam'
    # convoy_name_in_model = 'convoy'
    # fc1_name_in_model = 'follow_cart_1'
    # fc2_name_in_model = 'follow_cart_2'
    fc3_name_in_model = 'follow_cart_3'

    # 로봇 초기 생성 위치
    # convoy_spawn_x_val = '0.0'
    # convoy_spawn_y_val = '-2.0'
    # convoy_spawn_z_val = '0.1'
    # convoy_spawn_yaw_val = '0.0'

    # convoy yaw: 0.0 원점 기준

    # # 세로 대형: -1.0 가로 대형: -1.0 삼각 대형: -1.7
    # fc1_spawn_x_val = '-1.7'
    # # 세로 대형: 0.0 가로 대형: -1.0 삼각 대형: -1.0
    # fc1_spawn_y_val = '-1.0'
    # fc1_spawn_z_val = '0.1'
    # fc1_spawn_yaw_val = '0.0'

    # # 세로 대형: -2.0 가로 대형: -1.0 삼각 대형: -1.0
    # fc2_spawn_x_val = '-1.0'
    # # 세로 대형: 0.0 가로 대형: 0.0 삼각 대형: 0.0
    # fc2_spawn_y_val = '-2.0'
    # fc2_spawn_z_val = '0.1'
    # fc2_spawn_yaw_val = '0.0'

    # 세로 대형: -3.0 가로 대형: -1.0 삼각 대형: -1.7
    fc3_spawn_x_val = '-1.7'
    # 세로 대형: 0.0 가로 대형: +1.0 삼각 대형: +1.0
    fc3_spawn_y_val = '-3.0'
    fc3_spawn_z_val = '0.1'
    fc3_spawn_yaw_val = '0.0'

    # urdf 파일 경로
    pkg_share = get_package_share_directory(package_name)
    # convoy_urdf_path = os.path.join(pkg_share, 'urdf', 'convoy', 'jetbot.urdf')
    fc_urdf_path = os.path.join(pkg_share, 'urdf', 'follow_cart', 'turtlebot3_waffle_pi.urdf')

    # rviz 파일 경로
    rviz_config_file_path = os.path.join(pkg_share, 'rviz', 'mapper_rviz_config.rviz')

    # aws warehouse world 경로
    warehouse_pkg_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    warehouse_launch_path = os.path.join(warehouse_pkg_dir, 'launch')

    # gazebo 실행
    gazebo_run = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([warehouse_launch_path, '/no_roof_small_warehouse.launch.py'])
    )

    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    configuration_basename = 'cartographer.lua'

    # fc1 = LaunchConfiguration('fc1', default='fc1')
    # fc2 = LaunchConfiguration('fc2', default='fc2')
    fc3 = LaunchConfiguration('fc3', default='fc3')

    # convoy와 follow cart gazebo에 생성
    # spawn_convoy_cmd = Node(
    #     package='gazebo_ros',
    #     namespace='convoy',
    #     executable='spawn_entity.py',
    #     arguments=['-entity', convoy_name_in_model,
    #                '-topic', '/convoy/robot_description',
    #                '-x', convoy_spawn_x_val,
    #                '-y', convoy_spawn_y_val,
    #                '-z', convoy_spawn_z_val,
    #                '-Y', convoy_spawn_yaw_val],
    #     output='screen',
    #     parameters=[{'use_sim_time': True}],
    #     remappings=[
    #         ("/tf", "tf")])

    # spawn_fc1_cmd = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=['-entity', fc1_name_in_model,
    #                '-topic', '/fc1/robot_description',
    #                '-x', fc1_spawn_x_val,
    #                '-y', fc1_spawn_y_val,
    #                '-z', fc1_spawn_z_val,
    #                '-Y', fc1_spawn_yaw_val],
    #     output='screen',
    #     parameters=[{'use_sim_time': True}],
    #     remappings=[
    #         ("/tf", "tf")])

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
    #     parameters=[{'use_sim_time': True}],
    #     remappings=[
    #         ("/tf", "tf")])

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
        parameters=[{'use_sim_time': True}],
        remappings=[
            ("/tf", "tf")])

    # robot_state_publisher 실행
    # 로봇의 상태를 지속적으로 전달
    # convoy_state_publisher_cmd = Node(
    #     package='robot_state_publisher',
    #     namespace='convoy',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'use_sim_time': True,
    #                  'robot_description': Command(['xacro ', convoy_urdf_path])
    #                  }],
    #     remappings=[
    #         ("/tf", "tf"),
    #         ("/tf_static", "tf_static"),
    #         ("/robot_description", "robot_description")])

    # fc1_state_publisher_cmd = Node(
    #     package='robot_state_publisher',
    #     namespace='fc1',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'use_sim_time': True,
    #                  'robot_description': Command(['xacro ', fc_urdf_path, ' robot_name:=', fc1])}],
    #     remappings=[
    #         ("/tf", "tf"),
    #         ("/tf_static", "tf_static"),
    #         ("/robot_description", "robot_description")])

    # fc2_state_publisher_cmd = Node(
    #     package='robot_state_publisher',
    #     namespace='fc2',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'use_sim_time': True,
    #                  'robot_description': Command(['xacro ', fc_urdf_path, ' robot_name:=', fc2])}],
    #     remappings=[
    #         ("/tf", "tf"),
    #         ("/tf_static", "tf_static"),
    #         ("/robot_description", "robot_description")])

    fc3_state_publisher_cmd = Node(
        package='robot_state_publisher',
        namespace='fc3',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'robot_description': Command(['xacro ', fc_urdf_path, ' robot_name:=', fc3])}],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("/robot_description", "robot_description")])

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', configuration_basename],
        remappings=[
            ("/tf", "/fc3/tf"),
            ("/tf_static", "/fc3/tf_static"),
            ('/cmd_vel', '/fc3/cmd_vel'),
            ('/odom', '/fc3/odom'),
            ('/scan', '/fc3/scan'),
            ('/imu', '/fc3/imu'),
            ('/map', '/map')])

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        output='screen',
        name='occupancy_grid_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'])

    # rviz 실행
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file_path],
        remappings=[("/tf", "/fc3/tf"),
                    ("/tf_static", "/fc3/tf_static"),
                    ("/scan", "/fc3/scan"),
                    ("/odom", "/fc3/odom"),
                    ("/amcl_pose", "/fc3/amcl_pose"),
                    ("/robot_description", "/fc3/robot_description")])

    ld = LaunchDescription()

    ld.add_action(gazebo_run)
    # ld.add_action(spawn_convoy_cmd)
    # ld.add_action(convoy_state_publisher_cmd)
    # ld.add_action(fc1_state_publisher_cmd)
    # ld.add_action(spawn_fc1_cmd)
    # ld.add_action(fc2_state_publisher_cmd)
    # ld.add_action(spawn_fc2_cmd)
    ld.add_action(fc3_state_publisher_cmd)
    ld.add_action(spawn_fc3_cmd)

    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)

    ld.add_action(rviz)

    return ld
