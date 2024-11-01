from setuptools import setup

package_name = 'armmy_nodebridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Armmy2530',
    author_email='sutigran2557@gmail.com',
    description='odom bridge',
    license='Apache License 2.0',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'odom_bridge = armmy_nodebridge.odom_bridge:main',
            'delay_test = armmy_nodebridge.test_delay:main',
        ],
    },
)
