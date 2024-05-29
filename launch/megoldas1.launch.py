from launch import LaunchDescription,actions
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pid_controller',
            executable='pid_error.py',
            name='pid_fal_kovetes',
            output='screen'
        ),
        Node(
            package='pid_controller',
            executable='control.py',
            name='szabalyzo',
            output='screen'
        )
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
