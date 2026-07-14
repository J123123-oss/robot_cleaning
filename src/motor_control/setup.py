from setuptools import find_packages, setup

package_name = 'motor_control'

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
    maintainer='ubuntu',
    maintainer_email='a2723406795@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_control = motor_control.motor_control:main',
            'motor_driver = motor_control.motor_driver:main',
            'remote_control = motor_control.remote_control:main',
            'camera_remote_control_node = motor_control.camera_remote_control_node:main',
            'sensors_485 = motor_control.sensors_485:main',
            'laser_distance = motor_control.laser_distance:main',
            'charging = motor_control.charging:main',
            'manual_test = motor_control.manual_test:main',

        ],
    },
)
