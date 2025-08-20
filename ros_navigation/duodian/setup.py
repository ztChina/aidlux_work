from setuptools import find_packages, setup

package_name = 'duodian'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dingdian = duodian.dingdian:main',
            'ros_listener = duodian.ros_listener:main',
            'shiyan = duodian.shiyan:main',
            'yvying_jiaohu_daohang = duodian.yvying_jiaohu_daohang:main',
            'renti_shibi_daohang = duodian.renti_shibi_daohang:main',











        ],
    },
)
