from setuptools import find_packages, setup

package_name = 'onsen_dummy_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [
            'scripts/dummy_stream_node',
            'scripts/control_arbitrator_node',
            'scripts/arm_controller_node',
            'scripts/base_controller_node',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Onsen Robot',
    maintainer_email='chtsogbadrakh@gmail.com',
    description='Dummy ROS2 data publisher for onsen cleaning robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dummy_stream_node = onsen_dummy_robot.dummy_stream_node:main',
            'control_arbitrator_node = onsen_dummy_robot.control_arbitrator_node:main',
            'arm_controller_node = onsen_dummy_robot.arm_controller_node:main',
            'base_controller_node = onsen_dummy_robot.base_controller_node:main',
        ],
    },
)
