from setuptools import setup

package_name = 'alohax_hw_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.motors', f'{package_name}.motors.feetech', f'{package_name}.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/alohax_controllers.yaml', 'config/alohax_calibration.yaml']),
        ('share/' + package_name + '/launch', ['launch/hw_bridge.launch.py']),
    ],
    install_requires=['setuptools', 'deepdiff', 'tqdm', 'pyserial'],
    zip_safe=True,
    maintainer='Wolf',
    maintainer_email='wolf@example.com',
    description='Hardware bridge for AlohaX robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'alohax_bridge = alohax_hw_bridge.alohax_bridge:main',
        ],
    },
)
