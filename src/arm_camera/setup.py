from setuptools import find_packages, setup

setup(
    name='arm_camera',
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/arm_camera']),
        ('share/arm_camera', ['package.xml']),
    ],
    install_requires=['setuptools'],
    maintainer='TODO',
    maintainer_email='you@example.com',
    license='MIT',
    description='Publish USB camera images for the robotic arm.',
    entry_points={'console_scripts': ['camera_node = arm_camera.camera_node:main']},
)
