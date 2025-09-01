from setuptools import setup
from glob import glob
import os

package_name = 'g4_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='One-command bringup for Gazebo (headless), ros_gz bridge, Geant4 node, and viz nodes.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        # launch ファイルのみ。ノード追加は不要。
    },
)
