from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'pid_controller'

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
            # 'control_vehicle = pid_controller.control_vehicle:main',
            'pid_error.py = pid_controller.pid_error:main',
            'control.py = pid_controller.control:main',
            'simple_pursuit.py = pid_controller.simple_pursuit:main',
            'test.py = pid_controller.test:main'
        ],
    },
)
