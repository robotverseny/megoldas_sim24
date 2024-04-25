from launch import LaunchDescription
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
        # Node(
        #     package='rosservice',
        #     executable='rosservice',
        #     name='rosservice',
        #     arguments=['call', '/gazebo/reset_simulation']
        # )
    ])
