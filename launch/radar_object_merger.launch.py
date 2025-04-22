import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Generate launch description for radar_object_merger."""
    pkg_share = get_package_share_directory('radar_object_merger')
    param_file = os.path.join(pkg_share, 'config', 'radar_object_merger.param.yaml')

    # Declare launch arguments if needed for more flexibility, e.g., container name
    declare_container_name_arg = DeclareLaunchArgument(
        'container_name', default_value='radar_object_merger_container',
        description='Name of the composable node container'
    )
    declare_input_velocity_topic_arg = DeclareLaunchArgument(
        'input_velocity_topic', default_value='/vehicle/status/velocity_kmph', # Updated default topic
        description='Input topic for vehicle velocity (std_msgs/msg/Float32 in km/h)'
    )
    declare_output_objects_topic_arg = DeclareLaunchArgument(
        'output_objects_topic', default_value='/perception/object_recognition/radar/merged_objects',
        description='Output topic for merged radar objects'
    )

    # Define the ComposableNode
    radar_object_merger_node = ComposableNode(
        package='radar_object_merger',
        plugin='radar_object_merger::RadarObjectMerger',
        name='radar_object_merger_node',
        parameters=[param_file],
        remappings=[
            ('~/input/velocity', LaunchConfiguration('input_velocity_topic')),
            ('~/output/objects', LaunchConfiguration('output_objects_topic')),
            # Add other remappings if necessary
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # Define the ComposableNodeContainer
    container = ComposableNodeContainer(
        name=LaunchConfiguration('container_name'),
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            radar_object_merger_node,
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_container_name_arg,
        declare_input_velocity_topic_arg,
        declare_output_objects_topic_arg,
        container,
    ])
