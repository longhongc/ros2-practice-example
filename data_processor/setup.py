import os
from glob import glob
from setuptools import setup


package_name = 'data_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name+'.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
        (os.path.join('share', package_name), glob('rviz/*.rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chang-Hong Chen',
    maintainer_email='longhongc@gmail.com',
    description='Subscribers for sensors message',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'temp_subscriber = data_processor.temp_subscriber:main',
            'speed_subscriber = data_processor.speed_subscriber:main',
            'laser_subscriber = data_processor.laser_subscriber:main',
        ],
    },
)
