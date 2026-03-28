from setuptools import find_packages, setup

package_name = 'gimbal_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[ \
        'setuptools', 'rclpy', 'sensor_msgs', 'geometry_msgs', 'cv_bridge'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_detector = gimbal_tracker.aruco_detector:main',
            'camera_node = gimbal_tracker.camera_node:main',
        ],
    },
)
