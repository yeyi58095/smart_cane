from setuptools import setup
from glob import glob
import os

package_name = 'smart_cane_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ament 索引與 package.xml
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        # 安裝 launch/ 與 params/
        (os.path.join('share', package_name, 'launch'),
         glob('smart_cane_slam/launch/*.py')),
        (os.path.join('share', package_name, 'params'),
         glob('smart_cane_slam/params/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='yeyi58095@gmail.com',
    description='SLAM (slam_toolbox) launcher and params for smart_cane',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
