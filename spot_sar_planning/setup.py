import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'spot_sar_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'pddl'), glob('pddl/*.pddl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thanhnguyencanh',
    maintainer_email='canhthanhlt@gmail.com',
    description='Symbol grounding (world model) + PDDL domain/problem + unified-planning glue.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'world_model_node = spot_sar_planning.world_model_node:main',
            'floor_world_model_node = spot_sar_planning.floor_world_model_node:main',
        ],
    },
)
