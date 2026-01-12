from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'rtk_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob(os.path.join('custom_msgs', 'msg', '*.msg'))),
        (os.path.join('share', 'rtk_nav', 'launch'), [os.path.join('launch', f) for f in os.listdir('launch') if f.endswith('.launch.py')])

    ],
    install_requires=['setuptools', 'custom_msgs'],
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
            'cleaning_path_planner = rtk_nav.cleaning_path_planner:main',
            'three_point_planner = rtk_nav.three_point_planner:main',
            'wtrtk_parse_txt = rtk_nav.wtrtk_parse_txt:main',
            'wtrtk_serial_driver = rtk_nav.wtrtk_serial_driver:main',
            'rtk_nav = rtk_nav.rtk_nav:main',
        ],
    },


)
