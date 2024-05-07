from launch import LaunchDescription,actions
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz01',
            arguments=['-d', '$(find megoldas)/rviz/megoldas1.rviz']
        ),
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
        ),
        actions.ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/gazebo/reset_simulation', 'std_srvs/srv/Empty', '{}'],
            output='screen'
        )
    ])
