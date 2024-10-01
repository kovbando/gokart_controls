from setuptools import setup
import os
from glob import glob

package_name = 'gokart_controls'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch file in package
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Bandó Kovács, ELTE-IK',
    maintainer_email='kovbando@inf.elte.hu',
    description='This package contains helper nodes for the ELTEkart project',
    license='GLWTPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'speed_reader_node = gokart_controls.speed_reader_node:main',
        ],
    },
)
