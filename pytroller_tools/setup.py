from setuptools import find_packages, setup

package_name = 'pytroller_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maciej Bednarczyk',
    maintainer_email='mcbed.robotics@gmail.com',
    url="https://github.com/ICube-Robotics/pytroller",
    description='Tools for Pytrollers: Python controllers for ros2_control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          "generate_pxd = pytroller_tools.generate_pxd:main",
        ],
    },
)
