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

def generate_launch_description():

    package_name = 'follow_cart'
    convoy_name_in_model = 'convoy'
    fc1_name_in_model = 'follow_cart_1'
    fc2_name_in_model = 'follow_cart_2'
    fc3_name_in_model = 'follow_cart_3'
    default_launch_dir = 'launch'
    models_path = 'models'
    map_file_path = 'maps/map.yaml'
    convoy_nav2_params_path = 'params/convoy_nav2_object_following_params.yaml'
    fc1_nav2_params_path = 'params/follow_cart_1_nav2_object_following_params.yaml'
    convoy_localization_file_path = 'config/convoy_ekf.yaml'
    fc1_localization_file_path = 'config/follow_cart_ekf.yaml'
    rviz_config_file_path = 'rviz/convoy_and_fc1.rviz'
    convoy_sdf_path = 'models/convoy/model.sdf'
    fc1_sdf_path = 'models/follow_cart_1/model.sdf'
    fc2_sdf_path = 'models/follow_cart_2/model.sdf'
    fc3_sdf_path = 'models/follow_cart_3/model.sdf'
    convoy_urdf_file_path = 'urdf/convoy/jetbot.urdf'
    fc1_urdf_file_path = 'urdf/follow_cart/turtlebot3_waffle_pi.urdf'

    convoy_nav2_yaml = "convoy_amcl_config.yaml"
    fc1_nav2_yaml = "fc1_amcl_config.yaml"

    # Pose where we want to spawn the robot
    convoy_spawn_x_val = '0.0'
    convoy_spawn_y_val = '0.0'
    convoy_spawn_z_val = '1.0'
    convoy_spawn_yaw_val = '0.0'

    fc1_spawn_x_val = '-1.0'
    fc1_spawn_y_val = '0.0'
    fc1_spawn_z_val = '1.0'
    fc1_spawn_yaw_val = '0.0'

    fc2_spawn_x_val = '-1.0'
    fc2_spawn_y_val = '0.5'
    fc2_spawn_z_val = '1.0'
    fc2_spawn_yaw_val = '0.0'

    fc3_spawn_x_val = '-1.0'
    fc3_spawn_y_val = '-0.5'
    fc3_spawn_z_val = '0.5'
    fc3_spawn_yaw_val = '0.0'

    # Set the path to different files and folders.
    # pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_share = get_package_share_directory(package_name)
    default_launch_dir = os.path.join(pkg_share, default_launch_dir)
    convoy_urdf_model_path = os.path.join(pkg_share, convoy_urdf_file_path)
    fc1_urdf_model_path = os.path.join(pkg_share, fc1_urdf_file_path)
    convoy_localization_file_path = os.path.join(pkg_share, convoy_localization_file_path)
    fc1_localization_file_path = os.path.join(pkg_share, fc1_localization_file_path)
    default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)
    gazebo_models_path = os.path.join(pkg_share, models_path)
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    nav2_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_dir, 'launch')
    convoy_sdf_model_path = os.path.join(pkg_share, convoy_sdf_path)
    fc1_sdf_model_path = os.path.join(pkg_share, fc1_sdf_path)
    fc2_sdf_model_path = os.path.join(pkg_share, fc2_sdf_path)
    fc3_sdf_model_path = os.path.join(pkg_share, fc3_sdf_path)
    static_map_path = os.path.join(pkg_share, map_file_path)
    convoy_nav2_params_path = os.path.join(pkg_share, convoy_nav2_params_path)
    fc1_nav2_params_path = os.path.join(pkg_share, fc1_nav2_params_path)
    nav2_bt_path = get_package_share_directory('nav2_bt_navigator')

    warehouse_pkg_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    warehouse_launch_path = os.path.join(warehouse_pkg_dir, 'launch')

    # Launch configuration variables specific to simulation
    autostart = LaunchConfiguration('autostart')
    map_yaml = LaunchConfiguration('map_yaml')
    convoy_namespace = LaunchConfiguration('convoy_namespace')
    fc1_namespace = LaunchConfiguration('fc1_namespace')
    convoy_params_file = LaunchConfiguration('convoy_params_file')
    fc1_params_file = LaunchConfiguration('fc1_params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    convoy_sdf_model = LaunchConfiguration('convoy_sdf_model')
    fc1_sdf_model = LaunchConfiguration('fc1_sdf_model')
    fc2_sdf_model = LaunchConfiguration('fc2_sdf_model')
    fc3_sdf_model = LaunchConfiguration('fc3_sdf_model')
    slam = LaunchConfiguration('slam')
    convoy_urdf_model = LaunchConfiguration('convoy_urdf_model')
    fc1_urdf_model = LaunchConfiguration('fc1_urdf_model')
    use_namespace = LaunchConfiguration('use_namespace')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')


    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static")]

    # Declare the launch arguments
    declare_convoy_namespace_cmd = DeclareLaunchArgument(
        name='convoy_namespace',
        default_value='convoy',
        description='Top-level namespace')

    declare_fc1_namespace_cmd = DeclareLaunchArgument(
        name='fc1_namespace',
        default_value='fc1',
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

    declare_convoy_params_file_cmd = DeclareLaunchArgument(
        name='convoy_params_file',
        default_value=convoy_nav2_params_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_fc1_params_file_cmd = DeclareLaunchArgument(
        name='fc1_params_file',
        default_value=fc1_nav2_params_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

    declare_convoy_sdf_model_path_cmd = DeclareLaunchArgument(
        name='convoy_sdf_model',
        default_value=convoy_sdf_model_path,
        description='Absolute path to robot sdf file')

    declare_fc1_sdf_model_path_cmd = DeclareLaunchArgument(
        name='fc1_sdf_model',
        default_value=fc1_sdf_model_path,
        description='Absolute path to robot sdf file')

    declare_fc2_sdf_model_path_cmd = DeclareLaunchArgument(
        name='fc2_sdf_model',
        default_value=fc2_sdf_model_path,
        description='Absolute path to robot sdf file')

    declare_fc3_sdf_model_path_cmd = DeclareLaunchArgument(
        name='fc3_sdf_model',
        default_value=fc3_sdf_model_path,
        description='Absolute path to robot sdf file')

    declare_slam_cmd = DeclareLaunchArgument(
        name='slam',
        default_value='False',
        description='Whether to run SLAM')

    convoy_urdf_model_path_cmd = DeclareLaunchArgument(
        name='convoy_urdf_model',
        default_value=convoy_urdf_model_path,
        description='Absolute path to robot urdf file')

    fc1_urdf_model_path_cmd = DeclareLaunchArgument(
        name='fc1_urdf_model',
        default_value=fc1_urdf_model_path,
        description='Absolute path to robot urdf file')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Specify the actions

    gazebo_run = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([warehouse_launch_path, '/no_roof_small_warehouse.launch.py'])
        )

    # Launch the robot
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
        parameters=[{'use_sim_time': use_sim_time}])

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
        parameters=[{'use_sim_time': use_sim_time}])

    # spawn_fc2_cmd = Node(
    #     package='gazebo_ros',
    #     namespace='fc2',
    #     executable='spawn_entity.py',
    #     arguments=['-entity', fc2_name_in_model,
    #                '-file', fc2_sdf_model,
    #                '-x', fc2_spawn_x_val,
    #                '-y', fc2_spawn_y_val,
    #                '-z', fc2_spawn_z_val,
    #                '-Y', fc2_spawn_yaw_val],
    #     output='screen')
    #
    # spawn_fc3_cmd = Node(
    #     package='gazebo_ros',
    #     namespace='fc3',
    #     executable='spawn_entity.py',
    #     arguments=['-entity', fc3_name_in_model,
    #                '-file', fd3_sdf_model,
    #                '-x', fc3_spawn_x_val,
    #                '-y', fc3_spawn_y_val,
    #                '-z', fc3_spawn_z_val,
    #                '-Y', fc3_spawn_yaw_val],
    #     output='screen')

    # Start robot localization using an Extended Kalman filter
    convoy_localization_cmd = Node(
        package='robot_localization',
        namespace='convoy',
        executable='ekf_node',
        name='convoy_ekf_filter_node',
        output='screen',
        parameters=[convoy_localization_file_path,
                    {'use_sim_time': use_sim_time}],
        remappings=remappings)

    fc1_localization_cmd = Node(
        package='robot_localization',
        namespace='fc1',
        executable='ekf_node',
        name='fc1_ekf_filter_node',
        output='screen',
        parameters=[fc1_localization_file_path,
                    {'use_sim_time': use_sim_time}],
        remappings=remappings)

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    convoy_state_publisher_cmd = Node(
        package='robot_state_publisher',
        namespace='convoy',
        executable='robot_state_publisher',
        name='convoy_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', convoy_urdf_model]),
                     'frame_prefix': 'convoy/'
                     }],
        remappings=remappings)

    fc1_state_publisher_cmd = Node(
        package='robot_state_publisher',
        namespace='fc1',
        executable='robot_state_publisher',
        name='fc1_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', fc1_urdf_model]),
                     'frame_prefix': 'fc1/'}],
        remappings=remappings)

    convoy_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        namespace='convoy',
        executable='joint_state_publisher',
        name='convoy_joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', convoy_urdf_model])}],
        remappings=remappings)

    fc1_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        namespace='fc1',
        executable='joint_state_publisher',
        name='fc1_joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', fc1_urdf_model])}],
        remappings=remappings)



    # Launch RViz
    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        namespace='convoy',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        remappings=remappings
    )

    # Launch the ROS 2 Navigation Stack
    convoy_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': convoy_namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml,
                          'use_sim_time': use_sim_time,
                          'params_file': convoy_params_file,
                          'autostart': autostart}.items())

    fc1_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': fc1_namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml,
                          'use_sim_time': use_sim_time,
                          'params_file': fc1_params_file,
                          'autostart': autostart}.items())

    convoy_amcl = Node(
            namespace='convoy',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[convoy_nav2_yaml],
            remappings=remappings)

    fc1_amcl = Node(
        namespace='fc1',
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[fc1_nav2_yaml],
        remappings=remappings)

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'bond_timeout':0.0},
                    {'node_names': ['map_server', 'fc1/amcl', 'convoy/amcl']}])

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'topic_name':"map"},
                    {'frame_id':"map"},
                    {'yaml_filename':map_yaml}])


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

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_convoy_namespace_cmd)
    ld.add_action(declare_fc1_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_convoy_params_file_cmd)
    ld.add_action(declare_fc1_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_convoy_sdf_model_path_cmd)
    ld.add_action(declare_fc1_sdf_model_path_cmd)
    ld.add_action(declare_fc2_sdf_model_path_cmd)
    ld.add_action(declare_fc3_sdf_model_path_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(convoy_urdf_model_path_cmd)
    ld.add_action(fc1_urdf_model_path_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)


    # Add any actions
    ld.add_action(gazebo_run)
    ld.add_action(spawn_convoy_cmd)
    ld.add_action(spawn_fc1_cmd)
    # ld.add_action(spawn_follow_cart_2_cmd)
    # ld.add_action(spawn_follow_cart_3_cmd)
    ld.add_action(convoy_localization_cmd)
    ld.add_action(fc1_localization_cmd)
    ld.add_action(convoy_state_publisher_cmd)
    ld.add_action(fc1_state_publisher_cmd)
    ld.add_action(convoy_joint_state_publisher_cmd)
    ld.add_action(fc1_joint_state_publisher_cmd)

    ld.add_action(rviz)
    ld.add_action(lifecycle_manager)
    ld.add_action(map_server)
    ld.add_action(convoy_amcl)
    ld.add_action(fc1_amcl)

    # ld.add_action(convoy_controller)
    # ld.add_action(follower_controller)

    return ld