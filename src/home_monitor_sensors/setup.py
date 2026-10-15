from setuptools import setup

package_name = 'home_monitor_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    author='vboxuser',
    description='Sensor simulation for home monitoring',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = home_monitor_sensors.camera_node:main',
            'motion_sensor_node = home_monitor_sensors.motion_sensor_node:main',
        ],
    },
)
