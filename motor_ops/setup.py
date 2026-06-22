from setuptools import find_packages, setup

package_name = 'motor_ops'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samartha-s-newrro',
    maintainer_email='work@newrro.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'cmd_vel_sub = motor_ops.cmd_vel_sub:main',
            'mec_cmd_vel_sub = motor_ops.mec_cmd_vel_sub:main',
            'ticks_pub = motor_ops.ticks_pub:main',
	    'check_TPR = motor_ops.interactive_calibration:main'
        ],
    },
)
