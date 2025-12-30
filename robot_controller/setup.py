from setuptools import setup

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='apon',
    maintainer_email='apon@todo.todo',
    description='Robot controller node subscribing to tester UI commands.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'controller = robot_controller.main:main',
        ],
    },
)
