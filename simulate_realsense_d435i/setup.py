import os
from setuptools import setup
from glob import glob

package_name = 'simulate_realsense_d435i'

def package_files(directory):
    result = []
    for (path, directories, filenames) in os.walk(directory):
        paths = []
        for filename in filenames:
            paths.append(os.path.join(path, filename))
        result.append((os.path.join('share', package_name, path), paths))
    return result

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marina Banov',
    maintainer_email='m.banov7@gmail.com',
    description='This package uses a Turtlebot3 robot with an Intel Realsense D435i depth camera and tries to create pointclouds of Gazebo environments.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
