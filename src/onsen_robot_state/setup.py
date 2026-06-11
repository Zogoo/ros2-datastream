from setuptools import find_packages, setup

package_name = 'onsen_robot_state'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['scripts/robot_state_aggregator_node']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Onsen Robot',
    maintainer_email='chtsogbadrakh@gmail.com',
    description='Safety supervision and robot state fusion for the onsen robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_state_aggregator_node = onsen_robot_state.robot_state_aggregator_node:main',
        ],
    },
)
