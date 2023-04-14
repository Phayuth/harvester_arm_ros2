import os
from glob import glob

from setuptools import setup

package_name = 'onrobotsg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #use this to install launch.py so we call use : ros2 launch name_pkg name_launch https://roboticsbackend.com/ros2-launch-file-example/
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuth',
    maintainer_email='yuth@todo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'onrobotsg_service = onrobotsg.server:main',
        'onrobotsg_client = onrobotsg.client:main',
        ],
    },
)
