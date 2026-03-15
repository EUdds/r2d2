from setuptools import find_packages, setup

package_name = 'arm_ctlr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'adafruit-circuitpython-servokit',
        'lgpio',
    ],
    zip_safe=True,
    maintainer='eric',
    maintainer_email='udlis.eric@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arm_controller = arm_ctlr.arm_controller:main',
        ],
    },
)
