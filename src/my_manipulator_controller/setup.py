from setuptools import find_packages, setup

package_name = 'my_manipulator_controller'

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
    maintainer='ros2_user',
    maintainer_email='ros2_user@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manipulator_controller_server = my_manipulator_controller.my_manipulator_ctrl_srv:main',
            'manipulator_controller_client = my_manipulator_controller.my_manipulator_ctrl_cli:main',
        ],
    },
)
