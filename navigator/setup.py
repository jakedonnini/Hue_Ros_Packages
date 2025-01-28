from setuptools import find_packages, setup

package_name = 'navigator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hue',
    maintainer_email='jdonnini@seas.upenn.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node = navigator.navigation_node:main',
            'navigation_node_no_GPS = navigator.navigation_node_no_GPS:main',
<<<<<<< HEAD
            'navigation_node_RTOS = navigator.navigation_node_RTOS:main'
=======
            'TeleOp = navigator.TeleOp:main'
>>>>>>> 956467a05f6ce8e20f5bf97a5c8f651f0846339e
        ],
    },
)
