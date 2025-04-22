from setuptools import find_packages, setup

package_name = 'turtle_regulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vegeta',
    maintainer_email='vegeta@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'set_way_point = turtle_regulation.set_way_point:main',
            'waypoint_client = turtle_regulation.waypoint_client:main',
        ],
    },
)
