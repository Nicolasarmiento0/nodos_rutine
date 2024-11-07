import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mb1'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'chatbot'), glob(os.path.join('chatbot', '*.*'))),
        (os.path.join('share', package_name, 'chatbot/data'), glob(os.path.join('chatbot/data', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cmoralesd',
    maintainer_email='cear.inacap.cl@gmail.com',
    description='Mobile Base 1 (Develoment)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'modbus_bridge = mb1.modbus_bridge:main',
            'joy_velocity_publisher = mb1.joy_velocity_publisher:main',
            'led_strip_controller = mb1.led_strip_controller:main',
            'emotions = mb1.emotions:main',
            'chatbot = mb1.chatbot:main',
            'rutine_an4 = mb1.rutine_an4:main',

        ],
    },
)
