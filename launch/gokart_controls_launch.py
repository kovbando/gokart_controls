from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gokart_controls',  # Your package name
            executable='speed_reader_node',  # Your node executable
            name='serial_reader_node',
            parameters=[
                {'serial_port': '/dev/ttyACM0'},  # Default serial port
                {'baudrate': 9600},               # Default baudrate
                {'multiplier': 4.25}              # Default multiplier
            ]
        ),
    ])
