from setuptools import setup

package_name = 'armmy_nav2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Armmy2530',
    author_email='sutigran2557@gmail.com',
    description='NAV2 practice',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'example_nav_to_pose = armmy_nav2.example_nav_to_pose:main',
        ],
    },
)
