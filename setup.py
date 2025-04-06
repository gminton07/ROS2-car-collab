from setuptools import find_packages, setup

package_name = 'sample_pubsub'

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
    maintainer='Gabriel Minton',
    maintainer_email='gminton07@gmail.com',
    description='Sensors and control for use with PiCar',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub_imu      = sample_pubsub.pub_imu:main',
            'pub_keyboard = sample_pubsub.pub_keyboard:main',
            'pub_mmc5603  = sample_pubsub.pub_mmc5603:main',
            'listen3      = sample_pubsub.listen3:main',
        ],
    },
)
