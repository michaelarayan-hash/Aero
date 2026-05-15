from setuptools import find_packages, setup

package_name = 'camera_feed'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/precision_landing.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'camera_node = camera_feed.camera_node:main',
            'aruco_node = camera_feed.aruco_node:main',
            'landing_node = camera_feed.landing_node:main',
        ],
    },
)
