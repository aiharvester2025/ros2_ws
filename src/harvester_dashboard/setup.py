from setuptools import find_packages, setup

package_name = 'harvester_dashboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    # Zero ROS imports; runtime deps are the system PySide2/QtQuick,
    # python3-zmq, python3-msgpack, numpy, and Pillow provided by apt on
    # Ubuntu 20.04 (PySide2 5.14 has no QtQuickControls2 -> QtQuick 2 QML).
    install_requires=['setuptools'],
    package_data={'': ['../qml/*.qml']},
    zip_safe=False,
    maintainer='Oil Palm Harvester',
    maintainer_email='maintainer@example.com',
    description='Source-agnostic operator dashboard for canonical ZeroMQ telemetry v1 (view-only).',
    license='Apache-2.0',
    tests_require=['pytest'],
)
