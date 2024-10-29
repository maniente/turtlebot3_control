from setuptools import find_packages, setup

package_name = 'turtlebot3_control'

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
    maintainer='maniente',
    maintainer_email='marcomusto@tutanota.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go_to_goal = turtlebot3_control.go_to_goal:main',
            'go_forward = turtlebot3_control.ros2_goforward:main'
        ],
    },
)
