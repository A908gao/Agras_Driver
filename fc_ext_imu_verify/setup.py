import os
from glob import glob

from setuptools import setup

package_name = 'fc_ext_imu_verify'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FCCU Team',
    maintainer_email='fccu@example.com',
    description='FC (MAVROS) vs external MAVLink IMU attitude/position verification in RViz',
    license='MIT',
    entry_points={
        'console_scripts': [
            'verify_node = fc_ext_imu_verify.verify_node:main',
        ],
    },
)
