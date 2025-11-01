from setuptools import find_packages, setup

package_name = 'interpretOdom'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
                ('share/' + package_name + '/launch', ['launch/odometry.launch.py','launch/rtab_bot.launch.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yash',
    maintainer_email='yashc2025@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tracking_dead_wheel_node = interpretOdom.tracking_dead_wheel_odom:main',
            'two_tracking_dead_wheel_node = interpretOdom.two_tracking_dead_wheel_odom:main',
            'test_send_recieve_serial_json = interpretOdom.serial_data_send_recieve_test:main', 
            'new_pid_controller = interpretOdom.new_pid_controller:main',
            ],
    },
)
