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
            "thrusters_controller = control_pkg.thrusters_controller:main",
            "thrusters_velocity_mux = control_pkg.thrusters_velocity_mux:main"
        ],
    },
)
