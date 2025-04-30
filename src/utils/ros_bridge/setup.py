
#!/usr/bin/env python3

import os
from setuptools import setup

package_name = 'robometrics_bridge'

# Determine if we're in ROS1 or ROS2
ros_version = int(os.environ.get('ROS_VERSION', '1'))

if ros_version == 1:
    # ROS1 setup
    from catkin_pkg.python_setup import generate_distutils_setup
    
    d = generate_distutils_setup(
        packages=[package_name],
        package_dir={'': 'src'},
        scripts=['robometrics_bridge.py']
    )
    
    setup(**d)

else:
    # ROS2 setup
    from setuptools import find_packages
    
    setup(
        name=package_name,
        version='0.1.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            ('share/' + package_name + '/launch',
                ['launch_ros2/robometrics_bridge.launch.py']),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='RoboMonitor Team',
        maintainer_email='support@robometrics.xyz',
        description='RoboMonitor integration for ROS robots',
        license='MIT',
        entry_points={
            'console_scripts': [
                'robometrics_bridge = robometrics_bridge:main',
            ],
        },
    )
