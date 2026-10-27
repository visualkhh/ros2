from setuptools import setup

package_name = 'rl_correction'

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
    description='Movement correction RL node.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'corrector = rl_correction.corrector_node:main',
        ],
    },
)
