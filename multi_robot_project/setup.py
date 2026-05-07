from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'multi_robot_project'

def package_recursive_files(data_files, directory, install_base):
    for root, dirs, files in os.walk(directory):
        if files:
            install_path = os.path.join(install_base, root)
            file_list = [os.path.join(root, f) for f in files]
            data_files.append((install_path, file_list))
    return data_files

data_files_list = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        
        # World files
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        
        # Robot1
#        ('share/' + package_name + '/models/diffDriveBot1', glob('models/diffDriveBot1/*')),

        # Robot2
#        ('share/' + package_name + '/models/diffDriveBot2', glob('models/diffDriveBot2/*')),
        
        # Config files
        ('share/' + package_name + '/config', glob('config/*')),

    ]

data_files_list = package_recursive_files(data_files_list, 'models', os.path.join('share', package_name))

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
#    packages=[package_name],
    data_files=data_files_list,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maazshahzad',
    maintainer_email='maazsmughal@gmail.com',
    description='A ROS2 setup for multi-robot SLAM (hopefully Semantic) implemented in lieu of a course project.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    
    entry_points={
    'console_scripts': [
        'robot1_controller = multi_robot_project.robot1_controller:main',
        'robot2_controller = multi_robot_project.robot2_controller:main',
        'robot1_mapper_node = multi_robot_project.robot1_mapper:main',
        'robot2_mapper_node = multi_robot_project.robot2_mapper:main',
#        'mapper_node = multi_robot_project.mapper:main',
    ],
},
)


