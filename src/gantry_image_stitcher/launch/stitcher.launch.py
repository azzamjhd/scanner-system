from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_params_file = PathJoinSubstitution(
        [FindPackageShare('gantry_image_stitcher'), 'config', 'stitcher_params.yaml']
    )
    params_file = LaunchConfiguration('params_file')
    image_topic = LaunchConfiguration('image_topic')
    position_topic = LaunchConfiguration('position_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Parameter file for stitcher_node',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/image_raw',
            description='Camera image topic',
        ),
        DeclareLaunchArgument(
            'position_topic',
            default_value='/current_position',
            description='Gantry position topic in mm',
        ),
        Node(
            package='gantry_image_stitcher',
            executable='stitcher_node',
            name='stitcher_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'image_topic': image_topic,
                    'position_topic': position_topic,
                },
            ],
        ),
    ])
