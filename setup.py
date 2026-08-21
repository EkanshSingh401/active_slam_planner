import os
from glob import glob
from setuptools import setup

package_name = 'active_slam_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ssaigarimella',
    maintainer_email='sgarimella34@gatech.edu',
    description='Fisher-information-based active SLAM planning pipeline (scaffold).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fisher_ig_estimator = active_slam_planner.fisher_ig_estimator:main',
            'viewpoint_selector = active_slam_planner.viewpoint_selector:main',
            'trajectory_generator = active_slam_planner.trajectory_generator:main',
            'openvins_to_px4 = active_slam_planner.openvins_to_px4:main',
        ],
    },
)
