from glob import glob
from setuptools import find_packages, setup

setup(
    name='arm_gazebo', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/arm_gazebo']),
        ('share/arm_gazebo', ['package.xml']),
        ('share/arm_gazebo/launch', glob('launch/*.launch.py')),
        ('share/arm_gazebo/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
    maintainer='TODO', maintainer_email='you@example.com', license='MIT',
    description='Gazebo tabletop simulation for the robotic arm.',
    entry_points={'console_scripts': [
        'scene_manager = arm_gazebo.scene_manager:main',
    ]},
)
