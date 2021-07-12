from setuptools import setup

package_name = 'ros2_xacro_gazebo_spawner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Simon Honigmann',
    maintainer_email='simonhonigmann@gmail.com',
    description='A simple package that can dynamically spawn robots defined in xacro into Gazebo from ROS2 launch files',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_xacro_gazebo_spawner = ros2_xacro_gazebo_spawner.ros2_xacro_gazebo_spawner:main',
        ],
    },
)
