from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'stage_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # launch
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        #rviz config
        (os.path.join('share', package_name, 'config', 'rviz'),
            glob('config/rviz/*.rviz')),
        
        # worlds
        (os.path.join('share', package_name, 'world'),
            glob('world/*.world')),

        # includes dentro de world
        (os.path.join('share', package_name, 'world', 'include'),
            glob('world/include/*.inc')),
        
        # bitmaps
        (os.path.join('share', package_name, 'world', 'bitmaps'),
            glob('world/bitmaps/*')),
                
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
        ],
    },
)
