from setuptools import find_packages, setup

package_name = 'reference_signal_generator'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vtn21',
    maintainer_email='victor.noppeney@alumni.usp.br',
    description='This package implements a ROS node capable of publishing standard reference signals, mainly for use in robotic control systems. The services used to trigger the signals are based on the definitions of another package, reference_signal_srvs.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reference_signal_generator = reference_signal_generator.reference_signal_generator:main',
        ],
    },
)