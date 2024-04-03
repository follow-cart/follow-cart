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
    fc1_name_in_model = 'follow_cart_1'
    # fc2_name_in_model = 'follow_cart_2'
    # fc3_name_in_model = 'follow_cart_3'

    # 로봇 초기 생성 위치
    # convoy_spawn_x_val = '0.0'
    # convoy_spawn_y_val = '0.0'
    # convoy_spawn_z_val = '0.25'
    # convoy_spawn_yaw_val = '0.0'

    fc1_spawn_x_val = '-1.0'
    fc1_spawn_y_val = '0.0'
    fc1_spawn_z_val = '0.25'
    fc1_spawn_yaw_val = '0.0'

    # fc2_spawn_x_val = '-1.0'
    # fc2_spawn_y_val = '-2.5'
    # fc2_spawn_z_val = '0.25'
    # fc2_spawn_yaw_val = '0.0'
    #
    # fc3_spawn_x_val = '-1.0'
    # fc3_spawn_y_val = '-3.5'
    # fc3_spawn_z_val = '0.25'
    # fc3_spawn_yaw_val = '0.0'

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

    fc1 = LaunchConfiguration('fc1', default='fc1')
    # fc2 = LaunchConfiguration('fc2', default='fc2')
    # fc3 = LaunchConfiguration('fc3', default='fc3')

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

    spawn_fc1_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', fc1_name_in_model,
                   '-topic', '/fc1/robot_description',
                   '-x', fc1_spawn_x_val,
                   '-y', fc1_spawn_y_val,
                   '-z', fc1_spawn_z_val,
                   '-Y', fc1_spawn_yaw_val],
        output='screen',
        parameters=[{'use_sim_time': True}])

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
    #     parameters=[{'use_sim_time': True}],
    #     remappings=[
    #         ("/tf", "tf")])

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

    fc1_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'robot_description': Command(['xacro ', fc_urdf_path, ' robot_name:=', fc1])}],
        remappings=[
            ("/robot_description", "/fc1/robot_description")])

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

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', configuration_basename],
        remappings=[
            ('/cmd_vel', '/fc1/cmd_vel'),
            ('/odom', '/fc1/odom'),
            ('/scan', '/fc1/scan'), ])

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
        arguments=['-d', rviz_config_file_path]
    )

    ld = LaunchDescription()

    ld.add_action(gazebo_run)

    ld.add_action(fc1_state_publisher_cmd)
    ld.add_action(spawn_fc1_cmd)

    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)

    ld.add_action(rviz)

    return ld
