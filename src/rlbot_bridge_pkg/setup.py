from setuptools import setup

package_name = 'rlbot_bridge_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'AgentNode = rlbot_bridge_pkg.AgentNode:main'
        ],
    },
)
