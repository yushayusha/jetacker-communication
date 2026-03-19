from setuptools import find_packages, setup

package_name = 'telemetry_package'

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
    maintainer_email='prachi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'main_gui = telemetry_package.main_gui:main',
            'fsm_execute = telemetry_package.challenge_one_fsm:main',
            'fsm_info = telemetry_package.fsm_subscribers:main',
            'uav_info = telemetry_package.uav_subscribers:main',
            'ugv_info = telemetry_package.ugv_subscribers:main',
            'listener = telemetry_package.subscriber_member_function:main',
            'demo_subscriber = telemetry_package.demo_subscriber:main',
            'demo_publisher = telemetry_package.demo_publisher:main',

        ],
    },
)
