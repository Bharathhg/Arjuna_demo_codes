from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'arjuna'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('arjuna/launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('arjuna/config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('arjuna/rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('arjuna/maps/*')),
        (os.path.join('share', package_name, 'webpage'), glob('arjuna/webpage/*')),
        (os.path.join('share', package_name, 'database'), glob('arjuna/database/*')),
        (os.path.join('share', package_name, 'models/ml_model'), glob('arjuna/models/ml_model/*')),
        (os.path.join('share', package_name, 'models/dl_model'), glob('arjuna/models/dl_model/*')),
        (os.path.join('share', package_name, 'models/rl_model'), glob('arjuna/models/rl_model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Newrro Tech',
    maintainer_email='info@newrro.in',
    description='Arjuna autonomous robot ROS 2 package',
    license='MIT',
    entry_points={
        'console_scripts': [
            
            # OpenCV scripts
            'camera_pub = arjuna.scripts.opencv.camera_pub:main',
            'camera_sub = arjuna.scripts.opencv.camera_sub:main',
            'qr_recog = arjuna.scripts.opencv.qr_rec:main',
            'qr_tracking = arjuna.scripts.opencv.qr_tracking:main',
            'qr_ct = arjuna.scripts.opencv.qr_ct:main',
            
            # Navigation scripts
            'go_to_point_rviz = arjuna.scripts.navigation.go_to_point_rviz:main',
            'go_to_point_terminal = arjuna.scripts.navigation.go_to_point_terminal:main',
            'obstacle_avoidance = arjuna.scripts.navigation.obstacle_avoidance:main',
            'algorithm_selector = arjuna.scripts.navigation.algorithm_selector_rviz:main',
            'multi_point_rviz = arjuna.scripts.navigation.multi_point_rviz:main',
            'multi_point_terminal = arjuna.scripts.navigation.multi_point_terminal:main',
            
            # CPU/RAM monitoring
            'cpu_ram_pub = arjuna.scripts.cpu_ram.cpu_ram:main',
            
            # Web GUI launcher
            'web_gui = arjuna.scripts.web_gui:main',
        ],
    },
)