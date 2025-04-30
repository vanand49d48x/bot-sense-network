
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'robot_id',
            default_value=EnvironmentVariable('ROBOMETRICS_ROBOT_ID', default_value='robot-ros-001'),
            description='Robot ID for RoboMonitor platform'
        ),
        
        DeclareLaunchArgument(
            'api_key',
            default_value=EnvironmentVariable('ROBOMETRICS_API_KEY', default_value=''),
            description='API key for RoboMonitor platform'
        ),
        
        DeclareLaunchArgument(
            'api_url',
            default_value=EnvironmentVariable(
                'ROBOMETRICS_API_URL', 
                default_value='https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry'
            ),
            description='API endpoint URL for RoboMonitor platform'
        ),
        
        DeclareLaunchArgument(
            'update_rate',
            default_value='0.1',
            description='Rate to send telemetry in Hz (0.1 = every 10 seconds)'
        ),
        
        # Launch the bridge node
        Node(
            package='robometrics_bridge',
            executable='robometrics_bridge.py',
            name='robometrics_bridge',
            parameters=[{
                'robot_id': LaunchConfiguration('robot_id'),
                'api_key': LaunchConfiguration('api_key'),
                'api_url': LaunchConfiguration('api_url'),
                'update_rate': LaunchConfiguration('update_rate'),
            }],
            output='screen'
        )
    ])
