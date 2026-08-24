from setuptools import setup
import os
from glob import glob

package_name = 'medical_scanner_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='azzam',
    maintainer_email='azzamujahid214@gmail.com',
    description='Medical 3D scanner package with RPLiDAR and motor encoder',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scanner_3d_node = medical_scanner_pkg.scanner_3d_node:main',
            'scanner_gui = medical_scanner_pkg.scanner_gui_node:main',
            'voxel_filter_node = medical_scanner_pkg.voxel_filter_node:main',
        ],
    },
)
