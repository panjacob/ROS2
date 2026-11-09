from setuptools import find_packages, setup

package_name = 'arm_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='you@example.com',
    description='Gamepad (sensor_msgs/Joy) to joint velocity jog for the FluidNC bridge.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_teleop = arm_teleop.joy_teleop:main',
        ],
    },
)
