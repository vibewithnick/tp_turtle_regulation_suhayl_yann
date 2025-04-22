from setuptools import setup
import os
from glob import glob

package_name = 'turtle_regulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='suhayl',
    maintainer_email='suhaylnawool5@gmail.com',
    description='A turtle regulation package for ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'set_way_point = turtle_regulation.set_way_point:main',
            'waypoint_client = turtle_regulation.waypoint_client:main',
        ],
    },
)
