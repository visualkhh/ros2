from setuptools import setup
import os
from glob import glob

package_name = 'tb3_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pank',
    maintainer_email='pank@example.com',
    description='TurtleBot3-style bringup: OpenCR driver, robot model, RL split.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'opencr_driver = tb3_bringup.opencr_driver_node:main',
            'scan_resample = tb3_bringup.scan_resample_node:main',
            'state_manager = tb3_bringup.state_manager_node:main',
            'cmd_manager = tb3_bringup.cmd_manager_node:main',
            'frontier_explorer = tb3_bringup.frontier_explorer_node:main',
        ],
    },
)
