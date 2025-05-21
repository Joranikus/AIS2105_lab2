from setuptools import find_packages, setup

package_name = 'pid_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pid_controller_launch.py']),
        ('share/' + package_name + '/config', ['config/parameters.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orjan',
    maintainer_email='orjan.t.lynnes@ntnu.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_controller_node = pid_controller.pid_controller_node:main',
            'joint_simulator_node = joint_simulator.joint_simulator_node:main',
            'reference_input_node = pid_controller.reference_input_node:main',
        ],
    },
)
