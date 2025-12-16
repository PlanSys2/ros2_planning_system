from setuptools import find_packages, setup

package_name = 'plansys2_support_py'

setup(
    name=package_name,
    version='3.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Francisco Martin Rico',
    maintainer_email='fmrico@gmail.com',
    description='This package contains modules for developing PlanSys components in Python',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'planner = ' + package_name + '.Planner:main',
        ],
    },
)
