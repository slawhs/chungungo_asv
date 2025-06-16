from setuptools import find_packages, setup

package_name = 'perception_pkg'

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
    maintainer='grupo4',
    maintainer_email='grupo4@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "lidar_processing = perception_pkg.lidar_processing:main",
            "camera = perception_pkg.camera:main",
            "color_picker = perception_pkg.color_picker:main",
            "lidar = perception_pkg.lidar:main"
        ],
    },
)
