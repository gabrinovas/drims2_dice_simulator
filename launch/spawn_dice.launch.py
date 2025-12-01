from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'face_up',
            default_value='0',
            description='Face number facing upward (1-6, 0 = random)'
        ),
        DeclareLaunchArgument(
            'dice_size',
            default_value='0.037',
            description='Length of the dice edge (in meters)'
        ),
        DeclareLaunchArgument(
            'position',
            default_value='[0.25, 0.0, 0.80]',
            description='Initial dice position [x, y, z] in world frame'
        ),
        DeclareLaunchArgument(
            'dice_mesh',
            default_value='simplify_Die-OBJ.obj',
            description='OBJ mesh file to use (Dice.obj, Die-OBJ.obj, simplify_Die-OBJ.obj)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) time if true'
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RVIZ2 for visualization'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('drims2_dice_simulator'),
                'rviz',
                'dice_visualization.rviz'
            ]),
            description='Path to RVIZ config file'
        ),

        # Main dice spawner node
        Node(
            package='drims2_dice_simulator',
            executable='dice_spawner',
            name='dice_spawner',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'face_up': LaunchConfiguration('face_up'),
                'dice_size': LaunchConfiguration('dice_size'),
                'position': LaunchConfiguration('position'),
                'dice_mesh': LaunchConfiguration('dice_mesh'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }]
        ),

        # RVIZ2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=lambda context: LaunchConfiguration('launch_rviz').perform(context) == 'true',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }]
        ),
    ])


# Additional launch configurations for different use cases
def generate_research_launch_description():
    """Launch configuration optimized for research"""
    return LaunchDescription([
        DeclareLaunchArgument(
            'face_up',
            default_value='4',
            description='Fixed face for consistent experiments'
        ),
        DeclareLaunchArgument(
            'dice_size',
            default_value='0.037',
            description='Standard dice size'
        ),
        DeclareLaunchArgument(
            'position', 
            default_value='[0.3, 0.0, 0.85]',
            description='Central position for better visibility'
        ),
        DeclareLaunchArgument(
            'dice_mesh',
            default_value='Die-OBJ.obj',
            description='Detailed mesh for research'
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Always launch RVIZ for research'
        ),
        
        Node(
            package='drims2_dice_simulator',
            executable='dice_spawner',
            name='dice_spawner',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'face_up': LaunchConfiguration('face_up'),
                'dice_size': LaunchConfiguration('dice_size'),
                'position': LaunchConfiguration('position'),
                'dice_mesh': LaunchConfiguration('dice_mesh'),
            }]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('drims2_dice_simulator'),
                'rviz',
                'dice_visualization.rviz'
            ])]
        ),
    ])


# Quick test launch
def generate_test_launch_description():
    """Quick test launch with simple cube"""
    return LaunchDescription([
        DeclareLaunchArgument(
            'face_up',
            default_value='0',
            description='Random face'
        ),
        DeclareLaunchArgument(
            'position',
            default_value='[0.3, 0.0, 0.85]',
            description='Test position'
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='No RVIZ for quick tests'
        ),
        
        Node(
            package='drims2_dice_simulator',
            executable='dice_spawner',
            name='dice_spawner',
            output='screen',
            parameters=[{
                'face_up': LaunchConfiguration('face_up'),
                'dice_size': 0.037,
                'position': LaunchConfiguration('position'),
                'dice_mesh': 'simplify_Die-OBJ.obj',  # Lightweight for testing
            }]
        ),
    ])