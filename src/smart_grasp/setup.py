import os
from glob import glob
from setuptools import setup

package_name = 'smart_grasp'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='mdz',
    maintainer_email='mdz@todo.todo',
    description='Vision-based intelligent grasping for AgileX Piper arm (eye-in-hand D405)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detector_node = smart_grasp.detector_node:main',
            'handeye_tf_node = smart_grasp.handeye_tf_node:main',
            'tf_validator_node = smart_grasp.tf_validator_node:main',
            'grasp_executor_node = smart_grasp.grasp_executor_node:main',
            'param_tuner = smart_grasp.param_tuner:main',
        ],
    },
)
