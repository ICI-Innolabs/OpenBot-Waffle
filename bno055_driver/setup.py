from glob import glob
from setuptools import setup
import os

package_name = 'bno055_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reyespinr',
    maintainer_email='reyespinr@vcu.edu',
    description='ROS2 Driver Node for BNO055 IMU.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bno055_driver = bno055_driver.bno055_driver:main'
        ],
    },
)