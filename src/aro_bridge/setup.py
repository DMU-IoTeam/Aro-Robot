from setuptools import find_packages, setup

package_name = 'aro_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
         ['launch/aro_bridge.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'roslibpy',
    ],
    zip_safe=True,
    maintainer='ioteam',
    maintainer_email='river20s@naver.com',
    description='Bridge node: subscribe via roslibpy to Pi /test_string and republish locally.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = aro_bridge.bridge_node:main',
        ],
    },
)
