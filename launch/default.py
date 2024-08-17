import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Load config files
    config_dir = os.path.join(get_package_share_directory('mujoco_interface'), 'config')  # Get the path to the config directory
    control_params = os.path.join(config_dir, 'velocity_control.yaml')                    # Change this for different control modes, gains
    camera_params = os.path.join(config_dir, 'default_camera.yaml')
    sim_params = os.path.join(config_dir, 'default_sim.yaml')                             # Fix the typo in variable name

    return LaunchDescription([
        # Node configuration
        Node(
            package='mujoco_interface',
            executable='mujoco_interface_node',
            output='screen',
            parameters=[
                control_params,
                camera_params,
                sim_params,
                {'xml_path': '/home/woolfrey/workspace/colcon/src/interface_mujoco_ros2/test/scene.xml'} # CHANGE THIS
            ]
        )
    ])

