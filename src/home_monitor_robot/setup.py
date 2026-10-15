from setuptools import setup, find_packages

package_name = 'home_monitor_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/patrol_robot.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/patrol_robot.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    author='vboxuser',
    description='Mobile patrol robot for home monitoring',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_robot=home_monitor_robot.patrol_robot:main',
        ],
    },
)
