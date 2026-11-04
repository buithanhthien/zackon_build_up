from glob import glob
from setuptools import find_packages, setup

setup(
    name='dock_shape_sim', version='0.2.0',
    packages=find_packages(where='src'), package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/dock_shape_sim']),
        ('share/dock_shape_sim', ['package.xml', 'README.md', 'UPSTREAM.md']),
        ('share/dock_shape_sim/launch', glob('launch/*.launch.py')),
        ('share/dock_shape_sim/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=False,
    maintainer='Khoa', maintainer_email='khoa@example.com',
    description='V-shape docking backend for the Zackon hardware workspace',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'v_docking = dock_shape_sim.control.node:main',
        'dock_mission = dock_shape_sim.mission.node:main',
        'dock_calibration = dock_shape_sim.mission.calibration:main',
    ]},
)
