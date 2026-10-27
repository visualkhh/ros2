from setuptools import setup

package_name = 'nav2_rl_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pank',
    maintainer_email='pank@example.com',
    description='Nav2 controller correction RL node.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'controller_corrector = nav2_rl_controller.controller_corrector_node:main',
        ],
    },
)
