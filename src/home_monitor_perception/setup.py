from setuptools import setup

package_name = 'home_monitor_perception'

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
    description='AI perception for home monitoring',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_node = home_monitor_perception.detection_node:main',
        ],
    },
)
