from setuptools import find_packages, setup

package_name = 'f2m_Delivery_Robot'

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
    maintainer='truman',
    maintainer_email='mizan05j@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'delivery_robot_controller_node = f2m_Delivery_Robot.delivery_robot_controller_node:main',
        ],
    },
)
