from setuptools import setup

package_name = 'plansys2_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Francisco Martin Rico',
    maintainer_email='fmrico@gmail.com',
    description='This package contains various utilities for working with the ROS2 Planning System.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'horizontal_bar_chart = plansys2_utils.horizontal_bar_chart:main'
        ],
    },
)
