from setuptools import find_packages, setup


package_name = 'harvester_telemetry_contract'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
    ],
    # ROS/Jetson environments provide MessagePack through the system or the
    # active Conda environment.  Do not make pkg_resources resolve a PyPI
    # distribution at ros2-run time; it may use a different interpreter.
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Oil Palm Harvester',
    maintainer_email='maintainer@example.com',
    description='Canonical ZeroMQ telemetry v1 packing and validation helpers.',
    license='Apache-2.0',
    tests_require=['pytest'],
)
