import os
from glob import glob

from setuptools import find_namespace_packages, setup

package_name = 'Follow_Lane'

setup(
    name=package_name,
    version='0.0.0',
    # media/ enthaelt nur Bilder, kein Python -> nicht als Paket einsammeln
    packages=find_namespace_packages(include=[package_name, package_name + '.src']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/media/Battery',
            glob(package_name + '/media/Battery/*.png')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='felix.faass@web.de',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'control_driving = Follow_Lane.src.control_driving:main',
            'debug_view      = Follow_Lane.src.debug_view:main',
            'detect_lane     = Follow_Lane.src.detect_lane:main',
            'ingress_node    = Follow_Lane.src.ingress_node:main',
            'safety_stop     = Follow_Lane.src.safety_stop:main',
            'switch_control  = Follow_Lane.src.switch_control:main',
        ],
    },
)
