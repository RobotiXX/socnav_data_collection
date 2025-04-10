from setuptools import find_packages, setup

package_name = 'distributed_bag_recorder'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/host.launch.py', 'launch/robot.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nhat Le',
    maintainer_email='nhatleminh1997@gmail.com',
    description='ROS 2 package to control distributed bag recording across a laptop and robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'host_bag_service = distributed_bag_recorder.host_bag_service:main',
            'robot_bag_service = distributed_bag_recorder.robot_bag_service:main',
        ],
    },
)
