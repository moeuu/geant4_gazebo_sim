from setuptools import setup

package_name = 'g4_mapper'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/occupancy_mapper.launch.py',
            'launch/occupancy_with_rviz.launch.py',
        ]),
        ('share/' + package_name + '/rviz', [
            'rviz/edep_grid.rviz',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='g4_mapper_maintainer',
    maintainer_email='you@example.com',
    description='Build 2D heatmap (OccupancyGrid) from /odom and /g4/edep',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'occupancy_mapper = g4_mapper.occupancy_mapper:main',
        ],
    },
)
