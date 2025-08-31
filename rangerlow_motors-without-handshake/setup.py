from setuptools import setup

package_name = 'rangerlow_motors'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sip',
    maintainer_email='gleb.tedpoope@gmail.com',
    description='Driver for Ranger rover',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motors_node = rangerlow_motors.motors_node:main',
            'servo_node = rangerlow_motors.servo_node:main'
        ],
    },
)
