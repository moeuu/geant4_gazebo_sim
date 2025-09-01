from setuptools import setup

package_name = 'g4_viz'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # ament 索引用
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # launch ファイルをインストール
        ('share/' + package_name + '/launch', [
            'launch/edep_plotter.launch.py',
            'launch/edep_hist.launch.py',
            'launch/edep_grid.launch.py',
        ]),
        # world(SDF) もインストール
        ('share/' + package_name + '/worlds', [
            'worlds/minimal.sdf',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='g4',
    maintainer_email='noreply@example.com',
    description='Edep plot/hist/grid visualization nodes and launch files',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # 実行ファイル名 = モジュール:main
            'edep_plotter = g4_viz.edep_plotter_node:main',
            'edep_hist_node = g4_viz.edep_hist_node:main',
            'edep_grid_node = g4_viz.edep_grid_node:main',
        ],
    },
)
