from setuptools import setup

package_name = 'robot_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tester_ui.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='apon',
    maintainer_email='apon@todo.todo',
    description='Tkinter tester UI that publishes commands via ROS 2 topics.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tester_ui = robot_ui.tester_ui:main',
        ],
    },
)
