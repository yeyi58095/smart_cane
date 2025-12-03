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
        (
            os.path.join('share', 'ament_index', 'resource_index', 'packages'),
            [os.path.join('resource', package_name)]
        ),

        # ★ 必須：把 package.xml 安裝到 share/<pkg>
        (
            os.path.join('share', package_name),
            ['package.xml']
        ),

        # worlds：安裝到 share/<pkg>/worlds
        (
            os.path.join('share', package_name, 'worlds'),
            glob(os.path.join(package_name, 'worlds', '*.world'))
        ),

        # launch：安裝到 share/<pkg>/launch
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join(package_name, 'launch', '*.py'))
        ),

        # resource（可選）：方便檢查
        (
            os.path.join('share', package_name, 'resource'),
            [os.path.join('resource', package_name)]
        ),

        # ✅ URDF：安裝到 share/<pkg>/urdf
        (
            os.path.join('share', package_name, 'urdf'),
            glob(os.path.join(package_name, 'urdf', '*.urdf'))
        ),

        # ✅ Gazebo 模型：上層檔案（model.sdf、model.config 等）
        (
            os.path.join('share', package_name, 'models', 'turtlebot3_burger'),
            [
                os.path.join(package_name, 'models', 'turtlebot3_burger', 'model.sdf'),
                os.path.join(package_name, 'models', 'turtlebot3_burger', 'model-1_4.sdf'),
                os.path.join(package_name, 'models', 'turtlebot3_burger', 'model.config'),
            ]
        ),

        # ✅ Gazebo 模型：meshes 資料夾底下的檔案
        (
            os.path.join('share', package_name, 'models', 'turtlebot3_burger', 'meshes'),
            glob(os.path.join(package_name, 'models', 'turtlebot3_burger', 'meshes', '*'))
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='yeyi58095@gmail.com',
    description='Gazebo world and spawn launch files (ament_python)',
    license='MIT',
    entry_points={},
)
