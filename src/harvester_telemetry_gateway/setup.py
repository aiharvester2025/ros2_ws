from glob import glob
from setuptools import find_packages, setup


package_name = 'harvester_telemetry_gateway'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    # The gateway deliberately runs under the active ROS Python environment,
    # which already supplies these runtime modules on the Xavier.  Listing
    # third-party PyPI names here makes generated /usr/bin console scripts
    # reject the valid active Conda environment before the module can start.
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Oil Palm Harvester',
    maintainer_email='maintainer@example.com',
    description='Read-only ROS 2 simulation telemetry gateway for canonical ZeroMQ v1.',
    license='Apache-2.0',
    tests_require=['pytest'],
)
