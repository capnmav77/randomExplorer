from setuptools import setup, find_packages

package_name = 'my_object_localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neo',
    maintainer_email='its.mastermind77@gmail.com',
    description='Object localization package for ROS2',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'object_localization = my_object_localization.Runner:main',
        ],
    },
)