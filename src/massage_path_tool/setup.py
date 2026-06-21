from setuptools import setup, find_packages

setup(
    name='massage_path_tool',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'massage_path_tool.core': ['models/*.task'],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/massage_path_tool']),
        ('share/massage_path_tool', ['package.xml']),
    ],
    install_requires=['setuptools', 'open3d'],
    zip_safe=False,
    entry_points={
        'console_scripts': [
            'gui = massage_path_tool.main:main',
            'massage_path_translator = massage_path_tool.core.path_translator:main',
        ],
    },
)
