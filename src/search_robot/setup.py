from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'search_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@put.poznan.pl',
    description='Search robot node and action server',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detector = search_robot.detector:main',
            'search_server = search_robot.search_server:main',
            'search_client = search_robot.search_client:main',
        ],
    },
)
