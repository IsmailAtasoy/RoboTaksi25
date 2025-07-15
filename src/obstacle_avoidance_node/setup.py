from setuptools import find_packages, setup

package_name = 'obstacle_avoidance_node'

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
    maintainer='gunes',
    maintainer_email='gunes@todo.todo',
    description='Engelden kaçınma hareketi için ROS2 node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoidance = obstacle_avoidance_node.obstacle_avoidance_node:main',
            'advanced_obstacle_avoidance = obstacle_avoidance_node.advanced_obstacle_avoidance:main',
        ],
    },
)
