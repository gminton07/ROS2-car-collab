from setuptools import find_packages, setup
import os
from glob import glob    # Allows launch files to be read

package_name = 'pi_car'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Minton',
    maintainer_email='gminton07@gmail.com',
    description='Sensors and control for use with PiCar',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub_imu      = pi_car.pub_imu:main',
            'pub_mmc5603  = pi_car.pub_mmc5603:main',
            'pub_ultra    = pi_car.pub_ultra:main',
            'pub_mcp3008  = pi_car.pub_mcp3008:main',
            'pub_camera   = pi_car.pub_camera:main',
            'pub_keyboard = pi_car.pub_keyboard:main',
            'listen3      = pi_car.listen3:main',
            'controller   = pi_car.controller:main',
            'sub_motor    = pi_car.sub_motor:main',
            'sub_servo    = pi_car.sub_servo:main',
            'servocontrol = pi_car.ServoNode:main',
        ],
    },
)
