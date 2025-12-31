from setuptools import setup

package_name = 'smart_cane_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='yeyi58095@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_bed_detector = smart_cane_perception.color_bed_detector:main',
            'multi_bed_detector = smart_cane_perception.multi_bed_detector:main',
            'yolo_sign_detector = smart_cane_perception.yolo_sign_detector:main',
            'yolo_lidar_landmark = smart_cane_perception.yolo_lidar_landmark:main',
            'fix_yolo_shebang = smart_cane_perception.fix_yolo_shebang:main',
            'goto_landmark = smart_cane_perception.goto_landmark:main',
        ],
    },
)
