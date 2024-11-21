from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cherry_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch/'),
         glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='m',
    maintainer_email='98614364+DreamyStranger@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'true_pose = cherry_bot.Odometry.GroundTruth:main',
            'noisy_pose = cherry_bot.Odometry.Sensors.PoseSensor:main',
            'ekf_pose = cherry_bot.Odometry.EKF:main',
            'nav2goal = cherry_bot.Navigation.Navigation:main',
            'goals = cherry_bot.Navigation.GoalHandler:main',
            'simulation = cherry_bot.simulation:main'
        ],
    }, 
)
