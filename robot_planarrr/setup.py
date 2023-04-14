from setuptools import setup

package_name = 'robot_planarrr'

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
    maintainer_email='yuth',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # exe name       = package name.filename.main_function
            "planar_node_exe = robot_planarrr.planar_rr_node:main",
            "planar_fwdk_exe = robot_planarrr.planar_rr_pub:main",
            "planar_invk_ext = robot_planarrr.planar_rr_sub:main",
            "turtle_closeloop = robot_planarrr.turtle_pubsub:main"
        ],
    },
)
