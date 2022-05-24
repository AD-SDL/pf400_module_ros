from setuptools import setup

package_name = 'arm_driver_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Doga Ozgulbas and Alan Wang',
    maintainer_email='dozgulbas@anl.gov',
    description='Driver for the PF400 robot arm',
    url='https://github.com/AD-SDL/arm_driver_pkg',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'arm_driver = arm_driver_pkg.arm_driver:main_null',
        ]
    },
)
