from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mujoco_interface',
            executable='mujoco_interface_node',
            name='mujoco_interface_node',
            output='screen',
            parameters=[
                {'xml_location': '/home/woolfrey/workspace/mujoco_menagerie/kuka_iiwa_14/scene.xml'},
                {'publisher_name': 'joint_states'},
                {'subscriber_name': 'joint_commands'},
                {'proportional_gain': 20.0},
                {'derivative_gain': 2.0},
                {'integral_gain': 5.0},
                {'camera_focal_point': [0.0, 0.0, 0.5]},
                {'camera_distance': 2.5},
                {'camera_azimuth': 45.0},
                {'camera_elevation': -30.0},
                {'camera_orthographic': True},
                {'simulation_frequency' : 500},
                {'visualization_frequency' : 20}
            ]
        )
    ])
