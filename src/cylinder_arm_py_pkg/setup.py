from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cylinder_arm_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"],exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools', 'rclpy', 'kivy'],
    zip_safe=True,
    maintainer='kphan20',
    maintainer_email='knpmths@gmail.com',
    description='Python Nodes for my Cylinder Arm Project',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kivy_app = cylinder_arm_py_pkg.app:main',
            'hardware_service = cylinder_arm_py_pkg.hardware_service:main'
        ],
    },
)
