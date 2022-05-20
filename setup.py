<<<<<<< HEAD
from setuptools import setup

package_name = 'arm_driver_pkg'

setup(
    name=package_name,
    version='0.0.0',
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
    description='Driver for the OT2',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'arm_driver = arm_driver_pkg.arm_driver:main_null',
        ]
    },
=======
import os
from setuptools import find_packages, setup

# Single source of truth for version
version_ns = {}
with open(os.path.join("pf400_client", "version.py")) as f:
    exec(f.read(), version_ns)
version = version_ns['__version__']

with open('README.rst') as f:
    long_description = f.read()

install_requires = []
with open('requirements.txt') as reqs:
    for line in reqs.readlines():
        req = line.strip()
        if not req or req.startswith('#'):
            continue
        install_requires.append(req)

setup(
    name="pf400_client",
    description="Tooling for rapid deployment of automation tooling.",
    long_description=long_description,
    long_description_content_type='text/x-rst',
    url='https://github.com/AD-SDL/PF400_cobot',
    version=version,
    packages=find_packages(),
    install_requires=install_requires,
    python_requires=">=3.8",
    license='MIT',
    maintainer='',
    maintainer_email='',
    classifiers=[
        'Intended Audience :: Science/Research',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Operating System :: POSIX',
        'Operating System :: MacOS :: MacOS X',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
    ]
>>>>>>> 14b34ee7b8320acbdfc496b55bc9e8f7da337d79
)
