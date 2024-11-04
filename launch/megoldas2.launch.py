import os
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    follow_the_gap = Node(
        package='megoldas_sim24',
        executable='follow_the_gap.py',
        output='screen',
        parameters=[
            {
                # 'kozepiskola_neve': "Nem definialt kozepiskola",
                # 'kozepiskola_azonosito': "A00",
                # 'angle_range': 360,
                # 'velocity': 1.00,
                # 'car_length': 0.445,
                # 'wheelbase': 0.3187,
                # 'map_frame': 'odom_combined', ## not uses yet
                # 'laser_frame': 'laser',
                # 'base_frame': 'base_link',
            }
        ]
    )

    # Get the path to the included launch file
    package_share = FindPackageShare("megoldas_sim24").find("megoldas_sim24")
    # print("----" + package_share)
    launch_file_path = os.path.join(package_share, 'string_rviz_overlay.launch.py')

    str_overlay = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
        FindPackageShare("megoldas_sim24"), '/string_rviz_overlay.launch.py'])
    )

    # Check if the file exists #  TODO: this is not the best way to check, the launch may exist, but not working is still an issue
    if os.path.exists(launch_file_path):
        return LaunchDescription([
            follow_the_gap,
            str_overlay
        ])
    else:
        # Log or handle the case where the file does not exist
        return LaunchDescription([
            follow_the_gap,
            LogInfo(msg="Error: string_rviz_overlay.launch.py not found")  # Log a message instead of crashing
        ])