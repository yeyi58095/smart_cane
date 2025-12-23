from setuptools import setup
from glob import glob
import os

package_name = 'smart_cane_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],  # 對應 smart_cane_bringup/__init__.py
    data_files=[
        # ★ 必須：註冊到 ament 索引
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # ★ 必須：package.xml
        (os.path.join('share', package_name), ['package.xml']),
        # rviz/launch 安裝到 share/<pkg> 下
        (os.path.join('share', package_name, 'launch'), glob('smart_cane_bringup/launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('smart_cane_bringup/rviz/*.rviz')),
        (os.path.join('share', package_name, 'resource'), ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='yeyi58095@gmail.com',
    description='Bringup and RViz configs (ament_python)',
    license='MIT',
    entry_points={
        'console_scripts': [
        'initialpose_pub = smart_cane_bringup.initialpose_pub:main',
        'fix_yolo_shebang = smart_cane_bringup.fix_yolo_shebang:main',
    ],
    },
)
