from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sign_detection_img'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            os.path.join("share", package_name, "models"),
            glob("models/sign_detection.pt"),
        ),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='yutarop.storm.7@gmail.com',
    description='ROS2 package for sign detection using YOLOv8',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "sign_detection_img = sign_detection_img.sign_detection_img:main",
        ],
    },
)
