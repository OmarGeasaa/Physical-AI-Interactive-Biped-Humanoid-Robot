import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Find the path to your URDF file
    urdf_file = os.path.join(
        get_package_share_directory('biped_description'),
        'urdf',
        'biped.urdf'
    )

    # 2. Read the XML file into a string
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 3. Launch the robot_state_publisher node
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        )
    ])