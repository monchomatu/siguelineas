from setuptools import find_packages, setup

package_name = 'path_makers'

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
    maintainer='ramon',
    maintainer_email='moncho701@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        "path_draw = path_makers.nodes.path_draw:main",
        "path_publisher = path_makers.nodes.path_publisher:main",
        "path_publisher_debug = path_makers.nodes.path_publisher_debug:main",
        "Bit_Map = path_makers.planning.Bit_Map:main",
        "rrt_exp = path_makers.planning.rrt_exp:main"
        ],
    },
)
