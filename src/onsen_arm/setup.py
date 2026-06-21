from setuptools import find_packages, setup

package_name = 'onsen_arm'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/arm_description.launch.py']),
        ('lib/' + package_name, ['scripts/joint_bridge_node']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Onsen Robot',
    maintainer_email='chtsogbadrakh@gmail.com',
    description='Arm URDF + joint bridge + analytic IK model',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_bridge_node = onsen_arm.joint_bridge_node:main',
        ],
    },
)
