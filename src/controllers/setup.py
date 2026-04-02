from setuptools import find_packages, setup

package_name = 'controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/controllers/launch', ['launch/master.launch.py']),
        ('share/controllers/launch', ['launch/reactive_rrt.launch.py']),
        ('share/controllers/launch', ['launch/system.launch.py']),
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
        "lqr_final = controllers.lqr_final:main",
        "supervisor_final = controllers.supervisor_final:main",
        "supervisor = controllers.supervisor:main"
        ],
    },
)
