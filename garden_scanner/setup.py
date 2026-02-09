from setuptools import setup, find_packages 

package_name = 'garden_scanner'

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
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Garden Scanner Package with English logs',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_objects = garden_scanner.spawn_objects:main',
            'mission_control = garden_scanner.rectangular_scanner:main',
        ],
    },
)