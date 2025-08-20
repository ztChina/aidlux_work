from setuptools import find_packages, setup
import os

package_name = 'yolov5_check'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['resource/cutoff_yolov5s_sigmoid_w8a8.qnn229.ctx.bin']),
        # 安装launch文件
        (os.path.join('share', package_name, 'launch'), 
        ['launch/vision.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aidlux',
    maintainer_email='aidlux@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov5_check = yolov5_check.yolov5_check:main',
        ],
    },
)
