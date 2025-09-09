# rulebook_constrainer/setup.py
from setuptools import setup

package_name = 'rulebook_constrainer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        ('share/' + package_name + '/launch', ['launch/rulebook_constrainer.launch.py']),
    ],
    install_requires=['setuptools', 'requests'],  # ⬅️ 추가
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Rulebook -> costmap masks (with LLM semantic compiler)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rulebook_constrainer = rulebook_constrainer.rulebook_constrainer_node:main',
        ],
    },
)
