from setuptools import find_packages, setup

package_name = 'hello_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='A simple Hello ROS 2 package demonstrating basic node structure',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hello_node = hello_ros2.hello_node:main',
        ],
    },
)
