from setuptools import setup
from glob import glob
import os

package_name = 'rlbot_bridge_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '/testing_scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share',package_name,'launch'), ['launch/control_launch.py']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        
    ],
#    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mgiglia',
    maintainer_email='mgiglia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'AgentNode = rlbot_bridge_pkg.AgentNode:main',
            'ResetGameState = rlbot_bridge_pkg.testing_scripts.reset_game:main',
            'RlbotTester = rlbot_bridge_pkg.testing_scripts.rlbot_tester:main',
            'SetGains = rlbot_bridge_pkg.testing_scripts.set_gains:main',
        ],
    },
)
