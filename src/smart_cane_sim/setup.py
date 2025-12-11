from setuptools import setup
from glob import glob
import os

package_name = 'smart_cane_sim'

data_files = [
    # ★ 必須：註冊到 ament 索引
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

    # URDF：安裝到 share/<pkg>/urdf
    (
        os.path.join('share', package_name, 'urdf'),
        glob(os.path.join(package_name, 'urdf', '*.urdf'))
    ),
]

# ✅ 這裡動態把 models/ 底下的所有檔案裝進去（turtlebot3_burger, sign_A, sign_toilet... 全包）
models_src = os.path.join(package_name, 'models')

for root, _, files in os.walk(models_src):
    if not files:
        continue  # 沒檔案就略過（避免只有空資料夾）
    # root 例如： smart_cane_sim/models/sign_A/materials/textures
    # 我們想要的目標路徑是：share/smart_cane_sim/models/sign_A/materials/textures
    rel_path = os.path.relpath(root, package_name)  # 變成 models/...
    dest = os.path.join('share', package_name, rel_path)

    file_paths = [os.path.join(root, f) for f in files]
    data_files.append((dest, file_paths))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],  # 對應 smart_cane_sim/__init__.py
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='yeyi58095@gmail.com',
    description='Gazebo world and spawn launch files (ament_python)',
    license='MIT',
    entry_points={},
)
