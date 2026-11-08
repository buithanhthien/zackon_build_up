from glob import glob
from setuptools import setup


package_name = 'bounded_exploration_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Khoa',
    maintainer_email='khoa@example.com',
    description='Bounded autonomous exploration for Zackon',
    license='BSD-3-Clause',
    entry_points={'console_scripts': [
        'mission = bounded_exploration_sim.mission:main',
        'scan_merger = bounded_exploration_sim.scan_merger:main',
    ]},
)
