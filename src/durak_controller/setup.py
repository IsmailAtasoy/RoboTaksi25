from setuptools import find_packages, setup

package_name = 'durak_controller'

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
    maintainer='furkan',
    maintainer_email='furkan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'stop_sign_node = durak_controller.stop_sign_node:main',
        'l_park_node = durak_controller.l_park_node:main',
        'right_turn_node = durak_controller.right_turn_algorithm_node:main', # Bu satırı ekleyin
        'obstacle_response_node = durak_controller.obstacle_response_node:main',
        'advanced_obstacle_avoidance_node = durak_controller.advanced_obstacle_avoidance_node:main',
        ],
    },
)
