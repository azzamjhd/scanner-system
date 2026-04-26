from setuptools import find_packages, setup

package_name = 'massage_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/massage_points.yaml']),
    ],
    package_data={'': ['py.typed', 'models/*.task']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='azzam',
    maintainer_email='azzam@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pose_node = massage_perception.pose_node:main',
        ],
    },
)
