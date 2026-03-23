from setuptools import find_packages, setup

package_name = 'beast_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='don',
    maintainer_email='dwilliestyle@gmail.com',
    description='Package including teleops for robot control',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'keyboard_ctrl = beast_controller.keyboard_ctrl:main',
            'joy_teleop = beast_controller.joy_teleop:main'
        ],
    },
)
