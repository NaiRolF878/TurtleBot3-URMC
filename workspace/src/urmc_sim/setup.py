import os
from glob import glob

from setuptools import setup

package_name = 'urmc_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Florian',
    maintainer_email='florianadvertising@gmx.de',
    description='2D-Simulator fuer den TurtleBot3 Burger (URMRC 2026)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_node = urmc_sim.sim_node:main',
        ],
    },
)
