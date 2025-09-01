from setuptools import setup
from glob import glob
from pathlib import Path
import os

package_name = 'g4_bringup'

data_files = [
    # ament インデックスの登録
    ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
    # マニフェスト
    (f'share/{package_name}', ['package.xml']),
    # launch / robots / worlds を share に入れる（← ここが重要）
    (f'share/{package_name}/launch', glob('launch/*.launch.py')),
    (f'share/{package_name}/robots', glob('robots/*.sdf')),
    (f'share/{package_name}/worlds', glob('worlds/*.sdf')),
]

# Python パッケージディレクトリが無い場合でも build 通るように存在チェック
packages = [package_name] if Path(package_name).is_dir() else []

setup(
    name=package_name,
    version='0.0.2',
    packages=packages,
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='One-command bringup for Gazebo (headless), ros_gz bridge, Geant4 node, and viz nodes.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'robot_measurement_node = g4_bringup.robot_measurement_node:main',
        ],
    },
)

