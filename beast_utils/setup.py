from setuptools import find_packages, setup

package_name = 'beast_utils'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'battery_monitor = beast_utils.battery_monitor:main',
            'oled_display = beast_utils.oled_display:main',
            'safety_stop = beast_utils.safety_stop:main'
        ],
    },
)
