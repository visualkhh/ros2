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
            'mock_base = tb3_bringup.mock_base_node:main',
            'opencr_driver = tb3_bringup.opencr_driver_node:main',
        ],
    },
)
