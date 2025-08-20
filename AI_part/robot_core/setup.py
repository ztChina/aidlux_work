from setuptools import find_packages, setup
import os

package_name = 'robot_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),   
    #packages=['robot_core'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         # 安装launch文件
        (os.path.join('share', package_name, 'launch'), 
        ['launch/all_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aidlux',
    maintainer_email='2042868954@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_core_node = robot_core.robot_state:main',
        ],
    },
)
