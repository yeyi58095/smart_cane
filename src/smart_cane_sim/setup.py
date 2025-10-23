from setuptools import setup
from glob import glob
import os

package_name = 'smart_cane_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],  # 對應 smart_cane_sim/__init__.py
    data_files=[
        # ★ 必須：註冊到 ament 索引，讓 FindPackageShare 能找到這個包
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # ★ 必須：把 package.xml 安裝到 share/<pkg>
        (os.path.join('share', package_name), ['package.xml']),
        # world 與 launch 一起安裝到 share/<pkg> 下，ros2 launch 才能找到
        (os.path.join('share', package_name, 'worlds'), glob('smart_cane_sim/worlds/*.world')),
        (os.path.join('share', package_name, 'launch'), glob('smart_cane_sim/launch/*.py')),
        # （可選）把 resource/<pkg> 也複製一份到 share/<pkg>/resource 方便檢查
        (os.path.join('share', package_name, 'resource'), ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='yeyi58095@gmail.com',
    description='Gazebo world and spawn launch files (ament_python)',
    license='MIT',
    entry_points={},
)
