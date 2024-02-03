from setuptools import setup
from glob import glob
import os

package_name = 'simple_controller_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mgiglia',
    maintainer_email='mgiglia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'SimpleControllerNode = simple_controller_pkg.SimpleControllerNode:main',
            'ReferenceGeneratorNode = simple_controller_pkg.ReferenceGeneratorNode:main',
            'StanleyControllerNode = simple_controller_pkg.StanleyControllerNode:main'
        ],
    },
)
