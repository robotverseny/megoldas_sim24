from launch import LaunchDescription,actions
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pid_controller',
            executable='simple_pursuit.py',
            name='pid_fal_kovetes',
            output='screen'
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
