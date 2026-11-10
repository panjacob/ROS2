from setuptools import find_packages, setup


package_name = 'arm_sim'

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
    description='Joint motion simulator for the arm control panel.',
    license='MIT',
    entry_points={'console_scripts': ['joint_simulator = arm_sim.joint_simulator:main']},
)
