from setuptools import setup
import os
from glob import glob

package_name = 'drims2_dice_simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        # Install rviz config
        (os.path.join('share', package_name, 'rviz'), 
         glob('rviz/*.rviz')),
        # Install urdf meshes
        (os.path.join('share', package_name, 'urdf'), 
         glob('urdf/*.obj')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Novas',
    maintainer_email='gabriel.novas@aimen.es',
    description='Dice spawning and face detection service for research',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dice_spawner = drims2_dice_simulator.dice_spawner:main',
        ],
    },
)
