from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='minerva_odometry',
            namespace='minerva_odometry',
            executable='odometry',
            name='minerva_odom'
        )
    ])
    
