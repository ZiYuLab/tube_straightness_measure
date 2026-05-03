import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('tsm_gz_sim')
    world_sdf = os.path.join(pkg, 'sdf', 'measure_world.sdf')
    tube_sdf = os.path.join(pkg, 'sdf', 'straight_tube', 'straight_tube.sdf')
    default_params = os.path.join(pkg, 'config', 'sim_config.yaml')

    declare_tube_type_cmd = DeclareLaunchArgument(
        'tube_type',
        default_value='straight',
        description='Type of tube to measure',
    )
    declare_params_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to tsw_gz_sim_node parameters yaml',
    )

    # 启动 Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'-r {world_sdf}'}.items(),
    )

    # 生成钢管模型
    spawn_tube = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'measure_world',
            '-name', 'straight_tube',
            '-file', tube_sdf,
            ],
            output='screen',
        )

    ros_gz_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[
                {'config_file': os.path.join(pkg, 'config', 'topic_bridge_config.yaml')},
                {'use_sim_time': True},
            ],
            output='screen',
        )

    # x y z yaw pitch roll parent child
    tf_rgbd_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '1.0', '1.0', '-1.5708', '0.7854', '0', 'world', 'rgbd_1/camera_link/rgbd_1'],
        parameters=[{'use_sim_time': True}],
    )
    tf_rgbd_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '-1.0', '1.0', '1.5708', '0.7854', '0', 'world', 'rgbd_2/camera_link/rgbd_2'],
        parameters=[{'use_sim_time': True}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'rviz.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    tsm_gz_sim = Node(
        package='tsm_gz_sim',
        executable='tsm_gz_sim_node',
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(declare_tube_type_cmd)
    ld.add_action(declare_params_cmd)
    ld.add_action(gazebo)
    ld.add_action(spawn_tube)
    ld.add_action(ros_gz_bridge)
    ld.add_action(tf_rgbd_1)
    ld.add_action(tf_rgbd_2)
    ld.add_action(rviz)
    ld.add_action(tsm_gz_sim)
    return ld
