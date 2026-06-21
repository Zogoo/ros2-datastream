from setuptools import find_packages, setup

package_name = 'onsen_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [
            'scripts/static_tf_node',
            'scripts/localizer_node',
            'scripts/depth_scan_node',
            'scripts/nav_server_node',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Onsen Robot',
    maintainer_email='chtsogbadrakh@gmail.com',
    description='Autonomous navigation for the onsen robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_tf_node = onsen_nav.static_tf_node:main',
            'localizer_node = onsen_nav.localizer_node:main',
            'depth_scan_node = onsen_nav.depth_scan_node:main',
            'nav_server_node = onsen_nav.nav_server_node:main',
        ],
    },
)
