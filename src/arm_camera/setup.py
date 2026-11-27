from setuptools import find_packages, setup

package_name = 'arm_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Runs on Windows (started from WSL by arm.launch.py), not a ROS node.
        ('share/' + package_name + '/windows', ['windows/windows_camera_bridge.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='you@example.com',
    description='Cameras (OpenCV/V4L2 or stream URL) publishing raw and JPEG images.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = arm_camera.camera_node:main',
        ],
    },
)
