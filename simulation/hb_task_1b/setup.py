from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hb_task_1b'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*'))),
        # (os.path.join('share', package_name, 'scripts'), glob(os.path.join('scripts', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='tanishqchaudhary101010@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = hb_task_1b.controller:main',
            'service_node = hb_task_1b.service_node:main',
        ],
    },
)
