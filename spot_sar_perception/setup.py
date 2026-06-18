from setuptools import find_packages, setup

package_name = 'spot_sar_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thanhnguyencanh',
    maintainer_email='canhthanhlt@gmail.com',
    description='Victim detection + localization for Spot SAR: RGB-D -> /victims (VictimArray).',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detector_node = spot_sar_perception.detector_node:main',
        ],
    },
)
