from setuptools import setup

package_name = 'robot_controller_virtual'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch',
            [
                'launch/virtual_controller_rviz.launch.py',
                'launch/try_mechanism.launch.py',
            ],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='apon',
    maintainer_email='apon@todo.todo',
    description='Virtual controller node subscribing to TesterUICommand and publishing JointState for RViz.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'controller_virtual = robot_controller_virtual.main:main',
            'joint_state_mapper = robot_controller_virtual.joint_state_mapper:main',
        ],
    },
)
