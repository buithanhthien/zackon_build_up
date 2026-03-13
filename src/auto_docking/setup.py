from setuptools import setup
import os
from glob import glob

package_name = 'auto_docking'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thien',
    maintainer_email='thien@todo.todo',
    description='LiDAR reflective strip auto-docking system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dock_detector = auto_docking.dock_detector_node:main',
            'dock_controller = auto_docking.dock_controller_node:main',
        ],
    },
)
