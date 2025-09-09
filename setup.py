from setuptools import setup
import os
from glob import glob

package_name = 'rulebook_constrainer'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, f'{package_name}/utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), 
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot SW Engineer',
    maintainer_email='developer@example.com',
    description='ROS2 package for generating rule-based constraint masks from YAML rulebook for Nav2 integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rulebook_constrainer_node = rulebook_constrainer.rulebook_constrainer_node:main',
        ],
    },
)
