from setuptools import find_packages, setup
import glob

package_name = 'robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
        ('share/' + package_name + '/urdf', glob.glob('urdf/*.xacro')),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),            
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='st',
    maintainer_email='st@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zynq_node = robot.zynq_node:main',
            'merge_lidar_ultrasonic_node = robot.merge_lidar_ultrasonic_node:main',
            'person_receiver = robot.person_receiver:main',
            'mode_gui = robot.mode_gui:main',
            'person_receiver_exhib = robot.exhib_jetson:main',
            'slam_fusion = robot.slam_fusion:main',
        ],
    },
)
