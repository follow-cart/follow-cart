# Author: Addison Sears-Collins
# Date: November 9, 2021
# Description: Launch a two-wheeled robot using the ROS 2 Navigation Stack.
#              The spawning of the robot is performed by the Gazebo-ROS spawn_entity node.
#              The robot must be in both SDF and URDF format.
#              If you want to spawn the robot in a pose other than the default, be sure to set that inside
#              the nav2_params_path yaml file: amcl ---> initial_pose: [x, y, z, yaw]
#              The robot will follow a point that you click on the map (at a fixed distance away).
#              Use the Publish Point button in RViz to publish the point's location to the /goal_update topic
# https://automaticaddison.com

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    package_name = 'follow_cart'
    convoy_name_in_model = 'convoy'
    follow_cart_1_name_in_model = 'follow_cart_1'
    follow_cart_2_name_in_model = 'follow_cart_2'
    follow_cart_3_name_in_model = 'follow_cart_3'
    default_launch_dir = 'launch'
    models_path = 'models'
    map_file_path = 'maps/map.yaml'
    nav2_params_path = 'params/nav2_object_following_params.yaml'
    robot_localization_file_path = 'config/ekf.yaml'
    rviz_config_file_path = 'rviz/nav2_config_object_following.rviz'
    convoy_sdf_path = 'models/convoy/model.sdf'
    follow_cart_1_sdf_path = 'models/follow_cart_1/model.sdf'
    follow_cart_2_sdf_path = 'models/follow_cart_2/model.sdf'
    follow_cart_3_sdf_path = 'models/follow_cart_3/model.sdf'
    # world_file_path = 'worlds/empty.world'
    urdf_file_path = 'urdf/follow_cart/turtlebot3_waffle_pi.urdf'

    # Pose where we want to spawn the robot
    convoy_spawn_x_val = '0.0'
    convoy_spawn_y_val = '0.0'
    convoy_spawn_z_val = '1.0'
    convoy_spawn_yaw_val = '0.0'

    follow_cart_1_spawn_x_val = '-1.0'
    follow_cart_1_spawn_y_val = '0.0'
    follow_cart_1_spawn_z_val = '1.0'
    follow_cart_1_spawn_yaw_val = '0.0'

    follow_cart_2_spawn_x_val = '-1.0'
    follow_cart_2_spawn_y_val = '0.5'
    follow_cart_2_spawn_z_val = '1.0'
    follow_cart_2_spawn_yaw_val = '0.0'

    follow_cart_3_spawn_x_val = '-1.0'
    follow_cart_3_spawn_y_val = '-0.5'
    follow_cart_3_spawn_z_val = '0.5'
    follow_cart_3_spawn_yaw_val = '0.0'

    # Set the path to different files and folders.
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_share = get_package_share_directory(package_name)
    default_launch_dir = os.path.join(pkg_share, default_launch_dir)
    default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)
    robot_localization_file_path = os.path.join(pkg_share, robot_localization_file_path)
    default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)
    # world_path = os.path.join(pkg_share, world_file_path)
    gazebo_models_path = os.path.join(pkg_share, models_path)
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    nav2_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_dir, 'launch')
    convoy_sdf_model_path = os.path.join(pkg_share, convoy_sdf_path)
    follow_cart_1_sdf_model_path = os.path.join(pkg_share, follow_cart_1_sdf_path)
    follow_cart_2_sdf_model_path = os.path.join(pkg_share, follow_cart_2_sdf_path)
    follow_cart_3_sdf_model_path = os.path.join(pkg_share, follow_cart_3_sdf_path)
    static_map_path = os.path.join(pkg_share, map_file_path)
    nav2_params_path = os.path.join(pkg_share, nav2_params_path)
    nav2_bt_path = get_package_share_directory('nav2_bt_navigator')

    warehouse_pkg_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    warehouse_launch_path = os.path.join(warehouse_pkg_dir, 'launch')

    # Launch configuration variables specific to simulation
    autostart = LaunchConfiguration('autostart')
    headless = LaunchConfiguration('headless')
    map_yaml = LaunchConfiguration('map_yaml')
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    convoy_sdf_model = LaunchConfiguration('convoy_sdf_model')
    follow_cart_1_sdf_model = LaunchConfiguration('follow_cart_1_sdf_model')
    follow_cart_2_sdf_model = LaunchConfiguration('follow_cart_2_sdf_model')
    follow_cart_3_sdf_model = LaunchConfiguration('follow_cart_3_sdf_model')
    slam = LaunchConfiguration('slam')
    urdf_model = LaunchConfiguration('urdf_model')
    use_namespace = LaunchConfiguration('use_namespace')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        name='use_namespace',
        default_value='true',
        description='Whether to apply a namespace to the navigation stack')

    declare_autostart_cmd = DeclareLaunchArgument(
        name='autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        name='map_yaml',
        default_value=static_map_path,
        description='Full path to map file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        name='params_file',
        default_value=nav2_params_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

    declare_convoy_sdf_model_path_cmd = DeclareLaunchArgument(
        name='convoy_sdf_model',
        default_value=convoy_sdf_model_path,
        description='Absolute path to robot sdf file')

    declare_follow_cart_1_sdf_model_path_cmd = DeclareLaunchArgument(
        name='follow_cart_1_sdf_model',
        default_value=follow_cart_1_sdf_model_path,
        description='Absolute path to robot sdf file')

    declare_follow_cart_2_sdf_model_path_cmd = DeclareLaunchArgument(
        name='follow_cart_2_sdf_model',
        default_value=follow_cart_2_sdf_model_path,
        description='Absolute path to robot sdf file')

    declare_follow_cart_3_sdf_model_path_cmd = DeclareLaunchArgument(
        name='follow_cart_3_sdf_model',
        default_value=follow_cart_3_sdf_model_path,
        description='Absolute path to robot sdf file')

    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient')

    declare_slam_cmd = DeclareLaunchArgument(
        name='slam',
        default_value='False',
        description='Whether to run SLAM')

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot urdf file')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    # declare_world_cmd = DeclareLaunchArgument(
    #     name='world',
    #     default_value=world_path,
    #     description='Full path to the world model file to load')

    # Specify the actions

    # # Start Gazebo server
    # start_gazebo_server_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    #     condition=IfCondition(use_simulator),
    #     launch_arguments={'world': world}.items())

    # Start Gazebo client
    # start_gazebo_client_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    #     condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

    gazebo_run = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([warehouse_launch_path, '/no_roof_small_warehouse.launch.py'])
        )

    # Launch the robot
    spawn_convoy_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', convoy_name_in_model,
                   '-file', convoy_sdf_model,
                   '-x', convoy_spawn_x_val,
                   '-y', convoy_spawn_y_val,
                   '-z', convoy_spawn_z_val,
                   '-Y', convoy_spawn_yaw_val],
        output='screen')

    spawn_follow_cart_1_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', follow_cart_1_name_in_model,
                   '-file', follow_cart_1_sdf_model,
                   '-x', follow_cart_1_spawn_x_val,
                   '-y', follow_cart_1_spawn_y_val,
                   '-z', follow_cart_1_spawn_z_val,
                   '-Y', follow_cart_1_spawn_yaw_val],
        output='screen')

    spawn_follow_cart_2_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', follow_cart_2_name_in_model,
                   '-file', follow_cart_2_sdf_model,
                   '-x', follow_cart_2_spawn_x_val,
                   '-y', follow_cart_2_spawn_y_val,
                   '-z', follow_cart_2_spawn_z_val,
                   '-Y', follow_cart_2_spawn_yaw_val],
        output='screen')

    spawn_follow_cart_3_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', follow_cart_3_name_in_model,
                   '-file', follow_cart_3_sdf_model,
                   '-x', follow_cart_3_spawn_x_val,
                   '-y', follow_cart_3_spawn_y_val,
                   '-z', follow_cart_3_spawn_z_val,
                   '-Y', follow_cart_3_spawn_yaw_val],
        output='screen')

    # Start the navsat transform node which converts GPS data into the world coordinate frame
    # start_navsat_transform_cmd = Node(
    #     package='robot_localization',
    #     executable='navsat_transform_node',
    #     name='navsat_transform',
    #     output='screen',
    #     parameters=[robot_localization_file_path,
    #                 {'use_sim_time': use_sim_time}],
    #     remappings=[('imu', 'convoy/imu'),
    #                 ('gps/fix', 'convoy/gps'),
    #                 ('gps/filtered', 'gps/filtered'),
    #                 ('odometry/gps', 'odometry/gps'),
    #                 ('odometry/filtered', 'odometry/global')])

    # Start robot localization using an Extended Kalman filter...map->odom transform
    # start_robot_localization_global_cmd = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node_map',
    #     output='screen',
    #     parameters=[robot_localization_file_path,
    #                 {'use_sim_time': use_sim_time}],
    #     remappings=[('odometry/filtered', 'odometry/global'),
    #                 ('/set_pose', '/initialpose')])
    #
    # Start robot localization using an Extended Kalman filter...odom->base_footprint transform
    # start_robot_localization_local_cmd = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node_odom',
    #     output='screen',
    #     parameters=[robot_localization_file_path,
    #                 {'use_sim_time': use_sim_time}],
    #     remappings=[('odometry/filtered', 'odometry/local'),
    #                 ('/set_pose', '/initialpose')])

    # Start robot localization using an Extended Kalman filter
    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path,
                    {'use_sim_time': use_sim_time}])

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', urdf_model])}],
        remappings=remappings)

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[default_urdf_model_path]
    )

    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])

    # Launch the ROS 2 Navigation Stack
    start_ros2_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'autostart': autostart}.items())



    convoy_controller = Node(
        package='follow_cart',
        executable='convoy_controller',
        name='convoy_controller',
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_convoy_sdf_model_path_cmd)
    ld.add_action(declare_follow_cart_1_sdf_model_path_cmd)
    ld.add_action(declare_follow_cart_2_sdf_model_path_cmd)
    ld.add_action(declare_follow_cart_3_sdf_model_path_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    # ld.add_action(declare_world_cmd)


    # Add any actions
    # ld.add_action(start_gazebo_server_cmd)
    # ld.add_action(start_gazebo_client_cmd)

    ld.add_action(gazebo_run)
    # ld.add_action(spawn_convoy_cmd)
    ld.add_action(spawn_follow_cart_1_cmd)
    # ld.add_action(spawn_follow_cart_2_cmd)
    # ld.add_action(spawn_follow_cart_3_cmd)
    # ld.add_action(start_navsat_transform_cmd)
    # ld.add_action(start_robot_localization_global_cmd)
    # ld.add_action(start_robot_localization_local_cmd)
    ld.add_action(start_robot_localization_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    # ld.add_action(start_follow_cart_1_state_publisher_cmd)
    # ld.add_action(start_follow_cart_2_state_publisher_cmd)
    # ld.add_action(start_follow_cart_3_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_ros2_navigation_cmd)

    # ld.add_action(convoy_controller)

    return ld