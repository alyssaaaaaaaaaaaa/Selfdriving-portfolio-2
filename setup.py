from setuptools import find_packages, setup

package_name = 'odometry_pkg'

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
    maintainer='alyssa',
    maintainer_email='alyssa@example.com',
    description='Odometry package for self driving assignment',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_sim_node = odometry_pkg.encoder_sim_node:main',
            'odometry_node = odometry_pkg.odometry_node:main',
            'slam_node = odometry_pkg.slam_node:main',
        ],
    },
)