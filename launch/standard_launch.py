from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare Launch Arguments
        DeclareLaunchArgument('control_mode', default_value='VELOCITY'),
        DeclareLaunchArgument('xml_location', default_value='/home/woolfrey/workspace/mujoco_menagerie/kuka_iiwa_14/scene.xml'),
        DeclareLaunchArgument('publisher_name', default_value='joint_states'),
        DeclareLaunchArgument('subscriber_name', default_value='joint_commands'),
        DeclareLaunchArgument('proportional_gain', default_value='0.05'),
        DeclareLaunchArgument('integral_gain', default_value='0.01'),
        DeclareLaunchArgument('derivative_gain', default_value='0.0'),
        DeclareLaunchArgument('camera_focal_point', default_value='[0.0, 0.0, 0.5]'),
        DeclareLaunchArgument('camera_distance', default_value='2.5'),
        DeclareLaunchArgument('camera_azimuth', default_value='45.0'),
        DeclareLaunchArgument('camera_elevation', default_value='-30.0'),
        DeclareLaunchArgument('camera_orthographic', default_value='true'),
        DeclareLaunchArgument('simulation_frequency', default_value='500'),
        DeclareLaunchArgument('visualization_frequency', default_value='20'),

        # MuJoCo Interface Node
        Node(
            package='mujoco_interface',
            executable='mujoco_interface_node',
            output='screen',
            parameters=[{
                'control_mode': LaunchConfiguration('control_mode'),
                'xml_location': LaunchConfiguration('xml_location'),
                'publisher_name': LaunchConfiguration('publisher_name'),
                'subscriber_name': LaunchConfiguration('subscriber_name'),
                'proportional_gain': LaunchConfiguration('proportional_gain'),
                'derivative_gain': LaunchConfiguration('derivative_gain'),
                'integral_gain': LaunchConfiguration('integral_gain'),
                'camera_focal_point': LaunchConfiguration('camera_focal_point'),
                'camera_distance': LaunchConfiguration('camera_distance'),
                'camera_azimuth': LaunchConfiguration('camera_azimuth'),
                'camera_elevation': LaunchConfiguration('camera_elevation'),
                'camera_orthographic': LaunchConfiguration('camera_orthographic'),
                'simulation_frequency': LaunchConfiguration('simulation_frequency'),
                'visualization_frequency': LaunchConfiguration('visualization_frequency')
            }]
        )
    ])

