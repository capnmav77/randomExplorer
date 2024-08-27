from setuptools import find_packages, setup

package_name = 'my_explorer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
            ['launch/my_explorer_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neo',
    maintainer_email='itshelp@gmail.com',
    description='Package for exploring a 2D Grid World with TurtleBot3',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explorer = my_explorer.explorer:main',
        ],
    },
)
