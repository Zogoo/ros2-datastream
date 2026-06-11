from setuptools import find_packages, setup

package_name = 'onsen_ai_worker'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [
            'scripts/ai_worker_node',
            'scripts/mission_executor_node',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Onsen Robot',
    maintainer_email='chtsogbadrakh@gmail.com',
    description='OpenCV-based AI worker for onsen robot detection and planning',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ai_worker_node = onsen_ai_worker.ai_worker_node:main',
            'mission_executor_node = onsen_ai_worker.mission_executor_node:main',
        ],
    },
)
