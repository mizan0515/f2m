from setuptools import find_packages, setup

package_name = 'f2m_Central_Server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','pymysql','fastapi','uvicorn'],
    zip_safe=True,
    maintainer='truman',
    maintainer_email='mizan05j@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fastapi_server = f2m_Central_Server.fastapi_server:main',
            'order_manager_node = f2m_Central_Server.order_manager_node:main',
        ],
    },
)
