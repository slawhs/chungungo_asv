from setuptools import find_packages, setup

package_name = 'control_pkg'

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
    maintainer='slawhs',
    maintainer_email='ignagaja@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "joystick_velocity = control_pkg.joystick_velocity:main",
            "thrusters_driver = control_pkg.thrusters_driver:main",
            "thrusters_velocity_mux = control_pkg.thrusters_velocity_mux:main",
            "action_processing = control_pkg.action_processing:main",
            "distance_buoy_controller = control_pkg.distance_buoy_controller:main",
            "imu_velocity_controller = control_pkg.imu_velocity_controller:main",
            "relay_driver = control_pkg.relay_driver:main",
            "nav_channel_routine = control_pkg.nav_channel_routine:main"
        ],
    },
)
