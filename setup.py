from setuptools import find_packages, setup

package_name = 'mock_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mock_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carmi',
    maintainer_email='carmi@example.invalid',
    description='Manual ROS 2 mock bridge for testing the real Orchestrator.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_bridge_gui = mock_bridge.gui:main',
            # A keyboard console over the same node, for a terminal-only host
            # or a quick scripted check.
            'mock_bridge_console = mock_bridge.headless:main',
            # Two managed nodes that do nothing but follow their lifecycle.
            # Started by the launch file; `ros2 run` them for a quick check.
            'mock_bridge_control = mock_bridge.lifecycle_nodes:control_main',
            'mock_bridge_planner = mock_bridge.lifecycle_nodes:planner_main',
            'mock_bridge_mock1 = mock_bridge.lifecycle_nodes:mock1_main',
            'mock_bridge_mock2 = mock_bridge.lifecycle_nodes:mock2_main',
        ],
    },
)
