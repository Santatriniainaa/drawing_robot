from setuptools import setup
import os
from glob import glob

package_name = 'drawbot_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Santatriniaina RAZAFINDRAKOTO',
    maintainer_email='santatriniainaraz080@gmail.com',
    description='Drawbot control package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'draw_node = drawbot_control.draw_node:main',
            'marker_publisher = drawbot_control.marker_publisher:main',
            'xyz_control_gui = drawbot_control.xyz_control_gui:main',
        ],
    },
)
