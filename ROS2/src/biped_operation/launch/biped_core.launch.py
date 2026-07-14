import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Locate the URDF file inside the biped_description package
    urdf_file_path = os.path.join(
        get_package_share_directory('biped_description'),
        'urdf',
        'biped.urdf' # Make sure your file is named exactly this!
    )
    
    with open(urdf_file_path, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([
        
        # ==========================================
        # 1. URDF & TF TREE (biped_description)
        # ==========================================
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),

        # ==========================================
        # 2. HARDWARE BRIDGE (biped_gateway)
        # ==========================================
        Node(
            package='biped_gateway',
            executable='gateway_node',
            name='gateway_node',
            output='screen'
        ),

        # ==========================================
        # 3. KINEMATICS ENGINE (biped_kinematics)
        # ==========================================
        # Aggregator & Upper Body
        Node(
            package='biped_kinematics',
            executable='cmd_aggregator_node',
            name='cmd_aggregator_node',
            output='screen'
        ),
        Node(
            package='biped_kinematics',
            executable='upper_body_controller',
            name='upper_body_controller',
            output='screen'
        ),
        # --- Left Leg ---
        Node(
            package='biped_kinematics', executable='hip_ik_node', name='left_hip_ik',
            parameters=[{'leg_prefix': 'left'}], output='screen'
        ),
        Node(
            package='biped_kinematics', executable='hip_fk_node', name='left_hip_fk',
            parameters=[{'leg_prefix': 'left'}], output='screen'
        ),
        Node(
            package='biped_kinematics', executable='ankle_ik_node', name='left_ankle_ik',
            parameters=[{'leg_prefix': 'left'}], output='screen'
        ),
        Node(
            package='biped_kinematics', executable='ankle_fk_node', name='left_ankle_fk',
            parameters=[{'leg_prefix': 'left'}], output='screen'
        ),
        # --- Right Leg ---
        Node(
            package='biped_kinematics', executable='hip_ik_node', name='right_hip_ik',
            parameters=[{'leg_prefix': 'right'}], output='screen'
        ),
        Node(
            package='biped_kinematics', executable='hip_fk_node', name='right_hip_fk',
            parameters=[{'leg_prefix': 'right'}], output='screen'
        ),
        Node(
            package='biped_kinematics', executable='ankle_ik_node', name='right_ankle_ik',
            parameters=[{'leg_prefix': 'right'}], output='screen'
        ),
        Node(
            package='biped_kinematics', executable='ankle_fk_node', name='right_ankle_fk',
            parameters=[{'leg_prefix': 'right'}], output='screen'
        ),

        # ==========================================
        # 4. HIGH-LEVEL BRAIN & ODOMETRY 
        # ==========================================
        Node(
            package='biped_walking_controller',
            executable='biped_walking_node',
            name='biped_walking_node',
            output='screen'
        ),
        Node(
            package='biped_walking_controller',
            executable='swing_trajectory_generator',
            name='swing_trajectory_generator',
            output='screen'
        ),
        Node(
            package='biped_odometry',
            executable='odometry_node',
            name='biped_odometry_node',
            output='screen'
        ),

        # ==========================================
        # 5. AI AGENT (biped_ai_agent)
        # ==========================================
        Node(
            package='biped_ai_agent',
            executable='ai_node',
            name='biped_ai_agent_node',
            output='screen'
        ),

        # ==========================================
        # 6. SENSORS (Astra Pro & RPLiDAR A1M8)
        # ==========================================
        # RPLiDAR A1M8
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB0', # Update if your USB port changes
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True
            }],
            output='screen'
        ),
        # Orbbec Astra Pro
        Node(
            package='astra_camera',
            executable='astra_camera_node',
            name='astra_camera_node',
            parameters=[{
                'depth_align': True, 
                'publish_tf': False  # TF is handled by your URDF!
            }],
            output='screen'
        ),
    ])
