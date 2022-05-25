import os
from setuptools import setup, Command, find_packages

class CleanCommand(Command):
    """Custom clean command to tidy up the project root."""
    user_options = []
    def initialize_options(self):
        pass
    def finalize_options(self):
        pass
    def run(self):
        os.system('rm -vrf ./build ./dist ./*.pyc ./*.tgz ./*.egg-info')


package_name = 'arm_driver_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude= ['resources', 'test*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    python_requires=">=3.8",
    maintainer='Doga Ozgulbas and Alan Wang',
    maintainer_email='dozgulbas@anl.gov',
    description='Driver for the PF400 robot arm',
    url='https://github.com/AD-SDL/arm_driver_pkg.git', 
    license='MIT License',
    tests_require=['pytest'],
    entry_points={ 
        'console_scripts': [
             'arm_driver = arm_driver_pkg.arm_driver:main_null',
             'arm_listener = pf400_client.arm_listener:main_null',
             'rpl_pf400 = pf400_client.rpl_pf400:main_null',
             'pf400_client = pf400_client.pf400_client:main_null',
             'dummy_server = pf400_client.dummy_server:main_null',

        ]
    },
    classifiers=[
        'Intended Audience :: Science/Research',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Operating System :: POSIX',
        'Operating System :: MacOS :: MacOS X',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
    ],
)
