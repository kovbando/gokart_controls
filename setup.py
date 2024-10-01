from setuptools import find_packages, setup

package_name = 'gokart_controls'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
