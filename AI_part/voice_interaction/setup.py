from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'voice_interaction'

# def recursive_model_files():
#     files = []
#     for dirpath, dirnames, filenames in os.walk('resource/models'):
#         rel_dir = os.path.relpath(dirpath, 'resource/models')
#         dest_dir = os.path.join('share', package_name, 'models', rel_dir)
#         files.extend([(dest_dir, [os.path.join(dirpath, f)]) for f in filenames])
#     return files

# def recursive_wake_model_files():
#     files = []
#     for dirpath, dirnames, filenames in os.walk('resource/wake_model'):
#         rel_dir = os.path.relpath(dirpath, 'resource/wake_model')
#         dest_dir = os.path.join('share', package_name, 'wake_model', rel_dir)
#         files.extend([(dest_dir, [os.path.join(dirpath, f)]) for f in filenames])
#     return files

def recursive_data_files(source_dir, install_subdir):
    files = []
    for dirpath, _, filenames in os.walk(source_dir):
        rel_dir = os.path.relpath(dirpath, source_dir)
        dest_dir = os.path.join('share', package_name, install_subdir, rel_dir)
        for f in filenames:
            files.append((dest_dir, [os.path.join(dirpath, f)]))
    return files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装launch文件
        (os.path.join('share', package_name, 'launch'), 
        ['launch/voice_interaction_launch.py']),
    ]+ recursive_data_files('resource/models', 'models')
    + recursive_data_files('resource/wake_model', 'wake_model'),  # 注意不是 models/wake_model,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aidlux',
    maintainer_email='mouzhengyu2466@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'asr_node = voice_interaction.asr_node:main',
            'llm_node = voice_interaction.llm_node:main',
            'tts_node = voice_interaction.tts_node:main',
            'vadrecorder_node = vadrecorder_node.vadrecorder_node:main',
            'dialoguecoordinator_node = dialoguecoordinator_node.dialoguecoordinator_node:main',
        ],
    },
)
