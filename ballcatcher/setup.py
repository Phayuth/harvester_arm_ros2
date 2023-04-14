from setuptools import setup

package_name = 'ballcatcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            "fake_jointstate_pub = ballcatcher.fake_jointstate:main",
            "ballpose_local_pub  = ballcatcher.ball_pose_localization:main",
        ],
    },
)
