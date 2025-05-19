from setuptools import find_packages, setup
from glob import glob  # <----glob 모듈 추가
import os  # <----os 모듈 추가


package_name = 'auto_driving'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['auto_driving', 'auto_driving.*', 'test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/calibration/extrinsic_calibration',
            glob('calibration/extrinsic_calibration/*.yaml')),
        ('share/' + package_name + '/param/lane', glob('param/lane/*.yaml')),
    ],
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='ljhwan1997@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "extrinsic_camera_calib = auto_driving.extrinsic_camera_calibration:main",
            "intrinsic_camera_calib = auto_driving.intrinsic_camera_calibration:main",
            "lane_control = auto_driving.lane_control:main",
            "detect_lane = auto_driving.detect_lane:main",
            "fake_lane_center_pub = test.fake_lane_center_pub:main",
            "image_projection = auto_driving.image_projection:main",
            "image_compensation = auto_driving.image_compensation:main"
        ],
    },
)
