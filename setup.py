"""Setup script for key_teleop_ros package."""
from glob import glob
from setuptools import find_packages, setup

package_name = 'key_teleop_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('resource/' + package_name, glob('resource/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leander Stephen Desouza',
    maintainer_email='leanderdsouza1234@gmail.com',
    description='Package to control a robot using keyboard',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'key_drive = key_teleop_ros.key_drive:main',
        ],
    },
)
