from launch import LaunchDescription,actions
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='megoldas_sim24',
            executable='simple_pursuit.py',
            output='screen',
            parameters=[
                {
                    'kozepiskola_neve': "Nem definialt kozepiskola",
                    'kozepiskola_azonosito': "A00",
                    'angle_range': 360,
                    'velocity': 1.00,
                    'car_length': 0.445,
                    'wheelbase': 0.3187,
                    'map_frame': 'odom_combined',
                    'laser_frame': 'laser',
                }
            ]

        ),
        
        # actions.ExecuteProcess(
        #     cmd=['ros2', 'service', 'call', '/gazebo/reset_simulation', 'std_srvs/srv/Empty', '{}'],
        #     output='screen'
        # )
        # Node(
        #     package='ros2_service',
        #     executable='ros2 service',
        #     name='rosservice',
        #     arguments=['call', '/gazebo/reset_simulation']
        # )
    ])
