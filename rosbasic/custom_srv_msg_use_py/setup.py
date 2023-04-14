from setuptools import setup

package_name = 'custom_srv_msg_use_py'

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
    maintainer='ub20',
    maintainer_email='yuth@todo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'msg_pub = custom_srv_msg_use_py.msg_pub:main',
        'msg_sub = custom_srv_msg_use_py.msg_sub:main',
        'srv_srvr= custom_srv_msg_use_py.srv_server:main',
        'srv_clnt= custom_srv_msg_use_py.srv_client:main',
        ],
    },
)
