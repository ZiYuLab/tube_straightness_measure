import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('tsm_gz_sim')
    world_sdf = os.path.join(pkg, 'sdf', 'measure_world.sdf')
    default_params = os.path.join(pkg, 'config', 'sim_config.yaml')

    tube_type = LaunchConfiguration('tube_type').perform(context)
    if tube_type == 'bend':
        tube_sdf  = os.path.join(pkg, 'sdf', 'bend_tube', 'bend_tube.sdf')
        tube_name = 'bend_tube'
    else:
        tube_sdf  = os.path.join(pkg, 'sdf', 'straight_tube', 'straight_tube.sdf')
        tube_name = 'straight_tube'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'-r {world_sdf}'}.items(),
    )

    spawn_tube = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'measure_world',
            '-name', tube_name,
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
        parameters=[
            LaunchConfiguration('params_file'),
            {'tube_model_name': tube_name},
        ],
        output='screen',
    )

    return [gazebo, spawn_tube, ros_gz_bridge, tf_rgbd_1, tf_rgbd_2, rviz, tsm_gz_sim]


def generate_launch_description():
    pkg = get_package_share_directory('tsm_gz_sim')
    default_params = os.path.join(pkg, 'config', 'sim_config.yaml')

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               os.path.join(pkg, 'sdf')),
        DeclareLaunchArgument('tube_type', default_value='straight',
                              description='Type of tube: straight or bend'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Path to tsm_gz_sim_node parameters yaml'),
        OpaqueFunction(function=launch_setup),
    ])
