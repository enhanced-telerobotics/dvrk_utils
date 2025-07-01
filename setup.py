from setuptools import find_packages, setup
from glob import glob

package_name = 'dvrk_utils'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob('resource/*')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erie_lab',
    maintainer_email='sxy841@case.edu',
    description='Utilities for working with the dVRK (da Vinci Research Kit) in ROS 2.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_listener = dvrk_utils.tf_listener:main',
            'tf_publisher = dvrk_utils.tf_publisher:main',
            'init_robot = dvrk_utils.init_robot:main',
        ],
    },
)
