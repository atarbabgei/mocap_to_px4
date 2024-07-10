
import os
from glob import glob
from setuptools import setup

package_name = 'mocap_to_px4'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Atar Babgei',
    maintainer_email='a.babgei22@imperial.ac.uk',
    description='Remap motion capture topic to PX4 IMU',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mocap_remap_to_px4 = mocap_to_px4.mocap_remap_to_px4:main'
        ],
    },
)
