import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    xml_path = '/home/woolfrey/workspace/colcon/src/interface_mujoco_ros2/test/scene.xml'  # CHANGE THIS
    
    # Load config files
    config_dir = os.path.join(get_package_share_directory('mujoco_interface'), 'config')            # Get the path to the config directory
    control_params = os.path.join(config_dir, 'velocity_control.yaml')                              # Change this for different control modes, gains
    camera_params = os.path.join(config_dir, 'default_camera.yaml')                                 #

    return LaunchDescription([
    
        # Simulation settings
        DeclareLaunchArgument('xml_location', default_value=xml_path),
        DeclareLaunchArgument('publisher_name', default_value='joint_states'),
        DeclareLaunchArgument('subscriber_name', default_value='joint_commands'),
        DeclareLaunchArgument('simulation_frequency', default_value='500'),
        DeclareLaunchArgument('visualization_frequency', default_value='20'),

        # Load control gains and camera settings from YAML files
        Node(
            package='mujoco_interface',
            executable='mujoco_interface_node',
            output='screen',
            parameters=[
                control_params,
                camera_params,
                {
                    'xml_location': LaunchConfiguration('xml_location'),
                    'publisher_name': LaunchConfiguration('publisher_name'),
                    'subscriber_name': LaunchConfiguration('subscriber_name'),
                    'simulation_frequency': LaunchConfiguration('simulation_frequency'),
                    'visualization_frequency': LaunchConfiguration('visualization_frequency')
                }
            ]
        )
    ])

