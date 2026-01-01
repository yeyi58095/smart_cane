from setuptools import setup

package_name = 'smart_cane_landmarks'

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
            'goto_landmark = smart_cane_landmarks.goto_landmark:main',
            'landmark_visualizer = smart_cane_landmarks.landmark_visualizer:main',
        ],
    },
)
