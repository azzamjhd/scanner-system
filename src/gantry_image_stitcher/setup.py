from setuptools import setup
import os
from glob import glob

package_name = 'gantry_image_stitcher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='azzam',
    maintainer_email='azzamujahid214@gmail.com',
    description='Distance-triggered gantry image stitching with live preview and final mosaic output.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stitcher_node = gantry_image_stitcher.stitcher_node:main',
        ],
    },
)
