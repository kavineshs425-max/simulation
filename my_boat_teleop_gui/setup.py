from glob import glob

from setuptools import setup

package_name = 'my_boat_teleop_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kavinesh',
    maintainer_email='kavineshs@wroxai.com',
    description='Window keyboard teleop for my_boat (global keys, ON/OFF, live parameters)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['teleop_gui = my_boat_teleop_gui.gui_node:main']},
)
