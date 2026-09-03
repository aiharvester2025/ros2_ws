from glob import glob
from setuptools import find_packages, setup

package_name = 'harvester_dock'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Oil Palm Harvester',
    maintainer_email='maintainer@example.com',
    description='Interactive docking orchestrator: LiDAR sweep, live height/distance estimation, and autonomous dock/undock.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dock_orchestrator = harvester_dock.dock_orchestrator:main',
        ],
    },
)
