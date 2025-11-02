from setuptools import setup
import os
from glob import glob

package_name = 'my_mav_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('my_mav_pkg/launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raft-controller',
    maintainer_email='raft-controller@todo.todo',
    description='RealSense + WebSocket launcher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'send_arduino = my_mav_pkg.send_arduino:main',
        'camera_publisher = my_mav_pkg.camera_publisher:main',
            # (no nodes needed for this)
        ],
    },
)

