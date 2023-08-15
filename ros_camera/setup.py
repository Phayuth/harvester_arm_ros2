from setuptools import setup
from glob import glob
import os

package_name = 'ros_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        'crop_localize = ros_camera.rs_crop_localization:main',
        'perception_tf = ros_camera.tf_pub:main',
        'crop_harvest_pose = ros_camera.rs_crop_harvest_pose:main',
        ],
    },
)
