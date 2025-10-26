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
            'record_topic_node = telemetry_package.record_topic:main',
            'kill_switch_node = telemetry_package.kill_switch:main',
            'bag_recorder_node = telemetry_package.bag_recorder:main',
            'main_gui_node = telemetry_package.main_gui:main'
        ],
    },
)
