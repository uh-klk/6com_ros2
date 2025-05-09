from setuptools import find_packages, setup

package_name = 'robot_actions'

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
    maintainer='ros2user',
    maintainer_email='k.l.koay@herts.ac.uk',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_action_server=robot_actions.robot_action_server:main',
            'robot_action_client=robot_actions.robot_action_client:main',
            'ra_server_challange=robot_actions.robot_action_server_challenge:main',
            'ra_client_challange=robot_actions.robot_action_client_challenge:main',

        ],
    },
)
