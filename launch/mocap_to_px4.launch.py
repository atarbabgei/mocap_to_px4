"""
This launch file starts the computer-side ROS2 nodes for VICON data streaming.
"""

# Imports
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the launch arguments
    declare_server_arg = DeclareLaunchArgument(
        'server',
        default_value='192.168.0.100',
        description='VICON server IP address'
    )
    declare_buffer_size_arg = DeclareLaunchArgument(
        'buffer_size',
        default_value='256',
        description='Buffer size for VICON client'
    )
    declare_topic_namespace_arg = DeclareLaunchArgument(
        'topic_namespace',
        default_value='mocap',
        description='Namespace for VICON topics'
    )
    declare_uav_topic_arg = DeclareLaunchArgument(
        'uav_topic',
        default_value='/mocap/drone/drone',
        description='UAV subscription topic'
    )
    declare_px4_visual_odometry_topic_arg = DeclareLaunchArgument(
        'px4_visual_odometry_topic',
        default_value='/fmu/in/vehicle_visual_odometry',
        description='PX4 visual odometry topic'
    )

    # Initialize launch description
    launch_me = LaunchDescription([
        declare_server_arg,
        declare_buffer_size_arg,
        declare_topic_namespace_arg,
        declare_uav_topic_arg,
        declare_px4_visual_odometry_topic_arg
    ])

    # Parameters from launch configuration
    server = LaunchConfiguration('server')
    buffer_size = LaunchConfiguration('buffer_size')
    topic_namespace = LaunchConfiguration('topic_namespace')
    uav_topic = LaunchConfiguration('uav_topic')
    px4_visual_odometry_topic = LaunchConfiguration('px4_visual_odometry_topic')

    # Node for VICON data streaming
    vicon_streaming_node = Node(
        package='mocap_vicon_client',
        executable='vicon_client',
        output='screen',
        parameters=[
            {'server': server},
            {'buffer_size': buffer_size},
            {'namespace': topic_namespace}
        ]
    )

    # Node for passing the VICON data to the PX4
    px4_pubsub_node = Node(
        package='mocap_to_px4',  # Replace with your package name
        executable='mocap_remap_to_px4',  # Replace with the name of your executable
        name='mocap_remap_to_px4',
        parameters=[
            {'uav_topic': uav_topic},
            {'px4_visual_odometry_topic': px4_visual_odometry_topic}
        ]
    )

    # Add nodes to launch description
    launch_me.add_action(vicon_streaming_node)
    launch_me.add_action(px4_pubsub_node)

    return launch_me
