from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'megoldas_sim24'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Farraj007',
    maintainer_email='BarhamFarraj@icloud.com',
    description='TODO: Package description',
    license='GNU General Public License v3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_pursuit.py = megoldas_sim24.simple_pursuit:main',
            'follow_the_gap.py = megoldas_sim24.follow_the_gap:main',
            'joystick_teleop_node = megoldas_sim24.joystick_teleop:main',
        ],
    },
)
