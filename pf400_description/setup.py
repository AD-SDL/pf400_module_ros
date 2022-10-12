from setuptools import setup
import os
from glob import glob

package_name = 'pf400_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rafael Vescovi',
    maintainer_email='ravescovi@anl.gov',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pf400_joint_pub = pf400_description.TCSJointPub:main',
            'joint_publisher = pf400_description.joint_publisher:main'
        ],
    },
)