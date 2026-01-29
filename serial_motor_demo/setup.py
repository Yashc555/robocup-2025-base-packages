from setuptools import setup

package_name = 'serial_motor_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=True,
    maintainer='newans',
    maintainer_email='josh.newans@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'cmd_vel_to_pwm = serial_motor_demo.cmd_vel_to_pwm:main',
            'OpenLoop_controller = serial_motor_demo.OpenLoop_controller:main',
            'deadwheel_pid_controller = serial_motor_demo.deadwheel_pid_controller:main'
        ],
    },
)
