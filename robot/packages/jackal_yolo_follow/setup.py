from setuptools import find_packages, setup

package_name = 'jackal_yolo_follow'

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
    maintainer='robot',
    maintainer_email='tylerjhom@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_follower = jackal_yolo_follow.yolo_follower:main',
            'nav_to_pose_test = jackal_yolo_follow.nav_to_pose_test:main',
            'yolo_nav2_follower = jackal_yolo_follow.yolo_nav2_follower:main',
        ],
    },
)
