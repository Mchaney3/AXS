from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'mowbot_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, package_name + '.roboclaw']),  # Include roboclaw package
    package_dir={
        '': '.',  # Tells setuptools that the packages are located at the root
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='A ROS 2 package for controlling a differential drive robot with Roboclaw and PS4 teleop.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'roboclaw_control = mowbot_pkg.roboclaw_control:main',
        ],
    },
)
