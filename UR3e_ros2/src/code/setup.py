from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'code'

# Buscar automáticamente archivos pickle y otros tipos en 'code/cinematica_python'
cinematica_python_files = glob('code/cinematica_python/*.pickle') 
code_files = glob('code/*.pickle') + glob('code/*.txt')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, code_files),
        ('share/' + package_name + '/cinematica_python', cinematica_python_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david',
    maintainer_email='david@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur3_control = code.ur3_control:main',
            'ur3_control_2 = code.ur3_control_2:main',
            'system_exp_v7= code.cinematica_python.system_exp_v7:main',
            'system_exp_v6= code.system_exp_v6:main',
            'system_ros2_v1= code.system_ros2_v1:main',
            'system_ros2_v2= code.system_ros2_v2:main',
            'system_ros2_v3= code.system_ros2_v3:main',
            'system_ros2_v4= code.system_ros2_v4:main',
            'system_ros2_v5= code.system_ros2_v5:main',
            'system_ros2_v6= code.system_ros2_v6:main',
            'robot_publisher= code.robot_publisher:main',
            'robot_publisher_v2= code.robot_publisher_v2:main',
            'robot_publisher_v3= code.robot_publisher_v3:main',
            'robot_publisher_v4= code.robot_publisher_v4:main',
            'robot_publisher_v5= code.robot_publisher_v5:main',
            'test_node= code.test_node:main',
            'EndoWristTestPublisher= code.EndoWristTestPublisher:main',
            'system_experimento= code.system_experimento:main'
        ],
    }
)
