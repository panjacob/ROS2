from setuptools import find_packages, setup

package_name = 'arm_nanobee_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial', 'PyYAML'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='you@example.com',
    description='ROS 2 bridge to the NanoBee arm firmware.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nanobee_bridge = arm_nanobee_bridge.nanobee_bridge:main',
        ],
    },
)
