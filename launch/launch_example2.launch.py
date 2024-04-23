import launch
import launch_ros.actions


def generate_launch_description():
    node = launch_ros.actions.Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[
            {'port': 8765},
            {'address': '0.0.0.0'},
            {'tls': False},
            {'certfile': ''},
            {'keyfile': ''},
            #{'topic_whitelist': "'.*'"},
            {'max_qos_depth': 10},
            {'num_threads': 0},
            {'use_sim_time': False},
        ]
    )

    return launch.LaunchDescription([node])