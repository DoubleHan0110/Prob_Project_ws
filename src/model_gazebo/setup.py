from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'model_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='han',
    maintainer_email='doublehan0110@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vel_cmd_to_wheel = model_gazebo.vel_cmd_to_wheel:main',
            'ukf_odometry = model_gazebo.ukf_odometry:main',
            'ground_truth_pub = model_gazebo.ground_truth_pub:main',
        ],
    },
)
