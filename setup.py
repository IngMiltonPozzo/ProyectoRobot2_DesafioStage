import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'stage_challenge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'worlds', 'include'), glob('worlds/include/*.inc')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config', 'rviz'), glob('config/rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ingmiltonpozzo90@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'navigator = stage_challenge.navigator:main',
            'checkpoint_detector = stage_challenge.checkpoint_detector:main',
            'map_markers = stage_challenge.map_markers:main',
        ],
    },
)
