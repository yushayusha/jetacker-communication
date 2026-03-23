from setuptools import find_packages, setup

package_name = 'uav_controls'

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
            'area_scan_ros2 = uav_controls.area_scan_ros2:main',
            'aruco_detector_improved = uav_controls.aruco_detector_improved:main',
            'land_on_target_ros2 = uav_controls.land_on_target_ros2:main',
            'touchdown_controller = uav_controls.touchdown_controller:main',
            'uav_integrated_controller = uav_controls.uav_integrated_controller:main',
            'ugv_controller = uav_controls.ugv_controller:main',
        ],
    },
)
