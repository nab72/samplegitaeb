from setuptools import find_packages, setup

package_name = 'aeb_controller1'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	    'aeb_controller1 = aeb_controller1.aeb_controller1:main',
            'publish = aeb_controller1.publish:main',
            'aeb_node = aeb_controller1.aeb_node:main',
            'aeb_controller1_radar_only = aeb_controller1.aeb_controller_radar_only:main',
            'aeb_controller_sf = aeb_controller1.aeb_controller_sf:main',
            'aeb_new = aeb_controller1.aeb_new:main',
        ],
    },
)
