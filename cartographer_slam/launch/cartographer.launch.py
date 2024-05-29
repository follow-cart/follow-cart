import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():

    package_name = 'cartographer_slam'
    fc1_name_in_model = 'follow_cart_1'

    # 로봇 초기 생성 위치
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.0'


    # urdf 파일 경로
    pkg_share = get_package_share_directory(package_name)
    fc_pkg_share = get_package_share_directory("follow_cart")
    fc_urdf_path = os.path.join(fc_pkg_share, 'urdf', 'follow_cart', 'turtlebot3_waffle_pi.urdf')

    # rviz 파일 경로
    rviz_config_file_path = os.path.join(pkg_share, 'rviz', 'mapper.rviz')

    # aws warehouse world 경로
    warehouse_pkg_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    warehouse_launch_path = os.path.join(warehouse_pkg_dir, 'launch')

    # gazebo 실행
    gazebo_run = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([warehouse_launch_path, '/small_warehouse.launch.py'])
    )

    # cartographer config 파일 경로
    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    configuration_basename = 'cartographer.lua'

    fc1 = LaunchConfiguration('fc1', default='fc1')

    # 로봇 생성
    spawn_fc1_cmd = Node(
        namespace='fc1',
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', fc1_name_in_model,
                   '-topic', '/fc1/robot_description',
                   '-x', spawn_x_val,
                   '-y', spawn_y_val,
                   '-z', spawn_z_val,
                   '-Y', spawn_yaw_val],
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ("/tf", "tf")])

    # robot_state_publisher 실행
    # 로봇의 상태를 지속적으로 전달
    fc1_state_publisher_cmd = Node(
        package='robot_state_publisher',
        namespace='fc1',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'robot_description': Command(['xacro ', fc_urdf_path, ' robot_name:=', fc1])}],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("/robot_description", "robot_description")])

    # cartographer 노드 생성
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', configuration_basename],
        remappings=[
            ("/tf", "/fc1/tf"),
            ("/tf_static", "/fc1/tf_static"),
            ('/cmd_vel', '/fc1/cmd_vel'),
            ('/odom', '/fc1/odom'),
            ('/scan', '/fc1/scan'),
            ('/imu', '/fc1/imu'),
            ("/map", "/map"),])
    # cartographer map 형식을 nav2에서 사용할 수 있는 형태롤 변경하는 노드
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
        remappings=[("/tf", "/fc1/tf"),
                    ("/tf_static", "/fc1/tf_static"),
                    ("/scan", "/fc1/scan"),
                    ("/odom", "/fc1/odom"),
                    ("/map", "/map"),
                    ("/amcl_pose", "/fc1/amcl_pose")])

    ld = LaunchDescription()

    # launch description에 추가
    ld.add_action(gazebo_run)

    ld.add_action(fc1_state_publisher_cmd)
    ld.add_action(spawn_fc1_cmd)

    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)

    ld.add_action(rviz)

    return ld
