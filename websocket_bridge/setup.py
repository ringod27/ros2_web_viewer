from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'websocket_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include config and launch files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'roslibpy'],
    zip_safe=True,
    maintainer='masierra892',
    maintainer_email='miguel.sierra@lomby.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'websocket_bridge = websocket_bridge.websocket_bridge:main'
        ],
    },
)
