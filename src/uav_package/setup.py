from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'uav_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name , 'launch'),
        glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prachi',
    maintainer_email='prachi2004@tamu.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pixhawk_publisher = uav_package.pixhawk_publisher:main',
            'start_challenge1 = uav_package.start_challenge1:main',
            'start_challenge2 = uav_package.start_challenge2:main',
            'start_challenge3 = uav_package.start_challenge3:main',
            'start_nav2 = uav_package.start_nav2:main',
            'send_csv = uav_package.send_csv:main',
            'get_csv = uav_package.get_csv:main',
            'uav_trigger_c1 = uav_package.uav_trigger_c1:main',
            'uav_trigger_c2c3 = uav_package.uav_trigger_c2c3:main',
        ],
    },
)
