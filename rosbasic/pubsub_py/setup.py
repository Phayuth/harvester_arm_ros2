from setuptools import setup

package_name = 'pubsub_py'

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
    maintainer_email='ub20@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        # add this for python
        # name_of_exe_you_want = name_of_pkg.name_of_file:main
        'pub_py = pubsub_py.pub_py:main',
        'sub_py = pubsub_py.sub_py:main'
        ],
    },
)
