from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wauv_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'models', 'role_signs_v3'),
            glob(os.path.join('models', 'role_signs_v3', '*'))),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Lokesh Sai Dasari',
    maintainer_email='ldasari2@wisc.edu',
    description='Object detection + distance for competition props, published for the behavior tree',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'perception = wauv_perception.perception_node:main',
        ],
    },
)
