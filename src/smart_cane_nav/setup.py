from setuptools import setup
from glob import glob
import os

package_name = 'smart_cane_nav'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('smart_cane_nav/launch/*.py')),
        (os.path.join('share', package_name, 'params'), glob('smart_cane_nav/params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='yeyi58095@gmail.com',
    description='Nav2 (SLAM mode) for smart_cane',
    license='MIT',
    entry_points={'console_scripts': [
        'nav_cmd_vel_ui = smart_cane_nav.nav_cmd_vel_ui:main',
        'qt_cmd_vel_ui = smart_cane_nav.qt_cmd_vel_ui:main',
    ]},
)
