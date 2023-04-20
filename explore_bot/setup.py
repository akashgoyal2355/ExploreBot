import os
from glob import glob
from setuptools import setup

package_name = 'explore_bot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share/', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share/', package_name, 'worlds'), glob('worlds/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Akash',
    maintainer_email='akash.goyal2355@gmail.com',
    description='Explorer Robot',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'detector = explore_bot.color_detector_node:main',
        ],
},
)
