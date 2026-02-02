from pathlib import Path

from setuptools import find_packages, setup

package_name = 'gateway'
base_dir = Path(__file__).resolve().parent
repo_root = base_dir.parent
proto_src = repo_root / "firmware" / "libs" / "r2bus" / "proto" / "r2bus_pb2.py"
nodes_src = repo_root / "r2bus_codegen" / "nodes.json"

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/r2bus_bridge.launch.py']),
        ('share/' + package_name + '/proto', ['proto/r2bus_pb2.py']),
        ('share/' + package_name + '/config', ['config/nodes.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eric',
    maintainer_email='udlis.eric@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'api = gateway.api:main',
            'r2bus_gateway = gateway.r2bus_gateway:main',
            'web_bridge = gateway.web_bridge:main',
        ],
    },
)
