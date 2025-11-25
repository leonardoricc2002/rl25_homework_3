import os
from glob import glob
from setuptools import setup

package_name = 'force_land'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[py]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giuseppelucino',
    maintainer_email='your.email@example.com',
    description='Package for force land node logic.',
    license='TODO: License declaration',
    tests_require=['pytest'],
)
