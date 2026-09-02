from glob import glob
from setuptools import find_packages, setup

package_name = 'harvester_boom_plan'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Oil Palm Harvester',
    maintainer_email='maintainer@example.com',
    description='Read-only boom distance/angle/extension estimation and docking-safety watchdog.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'boom_plan_node = harvester_boom_plan.boom_plan_node:main',
            'tree_docking_estimate_node = harvester_boom_plan.tree_docking_estimate_node:main',
            'boom_docking_demo = harvester_boom_plan.boom_docking_demo:main',
        ],
    },
)
