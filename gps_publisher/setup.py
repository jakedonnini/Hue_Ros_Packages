from setuptools import find_packages, setup

package_name = 'gps_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch", ['launch/gps_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hue',
    maintainer_email='jdonnini@seas.upenn.edu',
    description='Publish GPS Data',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_node = gps_publisher.gps_node:main',
            'gps2_node = gps_publisher.gps2_node:main',
            'gps3_node = gps_publisher.gps3_node:main',
            
        ],
    },

)
